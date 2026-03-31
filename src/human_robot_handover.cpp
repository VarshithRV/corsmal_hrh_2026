#include <memory>
#include <functional>
#include <chrono>
#include <sstream>
#include <cmath>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "motion_planning_abstractions/bare_bones_moveit.hpp"
#include "motion_planning_abstractions/ee_pose_tracker.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#define HANDOVER_Z_THRESHOLD 0.3

struct SharedGraspPose
{
    std::mutex mtx;
    std::optional<Eigen::Isometry3d> pose;
};

bool is_object_ready(const std::shared_ptr<SharedGraspPose>& shared_pose)
{
    std::lock_guard<std::mutex> lock(shared_pose->mtx);
    return shared_pose->pose.has_value() &&
           shared_pose->pose->translation().z() >= HANDOVER_Z_THRESHOLD;
}

void until_object_ready(
    const std::shared_ptr<SharedGraspPose>& shared_pose,
    const rclcpp::Logger& logger)
{
    rclcpp::Rate rate(std::chrono::milliseconds(10));

    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(shared_pose->mtx);
            if (shared_pose->pose.has_value() &&
                shared_pose->pose->translation().z() >= HANDOVER_Z_THRESHOLD) {
                return;
            }
        }
        rate.sleep();
    }
}

void sub_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::shared_ptr<SharedGraspPose>& shared_pose)
{
    Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
    object_pose.translate(Eigen::Vector3d{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    });
    object_pose.rotate(Eigen::Quaterniond{
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    });

    Eigen::Isometry3d dtransform = Eigen::Isometry3d::Identity();
    dtransform.translate(Eigen::Vector3d{0.0, 0.0, 0.0});
    dtransform.rotate(Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0});

    std::lock_guard<std::mutex> lock(shared_pose->mtx);
    shared_pose->pose = object_pose * dtransform;
}

void grasp_pose_publisher_timer_cb(
    const std::shared_ptr<rclcpp::Clock>& wall_clock,
    const std::shared_ptr<SharedGraspPose>& shared_pose,
    const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>& publisher)
{
    Eigen::Isometry3d pose_copy;

    {
        std::lock_guard<std::mutex> lock(shared_pose->mtx);
        if (!shared_pose->pose.has_value()) {
            return;
        }
        pose_copy = *shared_pose->pose;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = wall_clock->now();

    msg.pose.position.x = pose_copy.translation().x();
    msg.pose.position.y = pose_copy.translation().y();
    msg.pose.position.z = pose_copy.translation().z();

    Eigen::Quaterniond q(pose_copy.rotation());
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();

    publisher->publish(msg);
}

void callback(
    rclcpp::Node::SharedPtr node,
    std_srvs::srv::Trigger::Response::SharedPtr res,
    std::shared_ptr<BareBonesMoveit> single_arm_control_interface,
    std::shared_ptr<PoseTracker> ee_pose_tracker_interface,
    std::shared_ptr<SharedGraspPose> shared_pose)
{
    auto logger = node->get_logger();

    RCLCPP_INFO(logger, "Initiating handover sequence");

    single_arm_control_interface->move_to_joint_positions(
        {-0.251881, -1.21652, -2.01470, -3.057, -2.2568, 0.076789});

    {
        std::lock_guard<std::mutex> lock(shared_pose->mtx);
        if (!shared_pose->pose.has_value()) {
            RCLCPP_ERROR(logger, "No grasp pose received yet");
            res->success = false;
            res->message = "No grasp pose received yet";
            return;
        }
    }

    RCLCPP_INFO(logger, "Saw the object, now preparing the tracker");

    ee_pose_tracker_interface->prepare_tracker_();
    ee_pose_tracker_interface->clear_target_pose_();

    auto rate = rclcpp::Rate(std::chrono::milliseconds(50));

    geometry_msgs::msg::Pose start_pose = *single_arm_control_interface->get_current_ee_pose();
    geometry_msgs::msg::Pose current_pose(start_pose);

    ee_pose_tracker_interface->set_target_pose_(start_pose);
    ee_pose_tracker_interface->start_tracking_();

    auto get_position_error = [](const geometry_msgs::msg::Pose& pose1,
                                 const geometry_msgs::msg::Pose& pose2) {
        return Eigen::Vector3d{
            pose1.position.x - pose2.position.x,
            pose1.position.y - pose2.position.y,
            pose1.position.z - pose2.position.z
        };
    };

    auto pose_from_isometry = [](const Eigen::Isometry3d& tf) {
        geometry_msgs::msg::Pose pose;
        Eigen::Quaterniond q(tf.rotation());

        pose.position.x = tf.translation().x();
        pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        return pose;
    };

    auto vector_from_pose = [](const geometry_msgs::msg::Pose& pose) {
        return Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z};
    };

    auto quaternion_from_pose = [](const geometry_msgs::msg::Pose& pose) {
        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        q.normalize();
        return q;
    };

    auto make_pose = [](const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x();
        pose.position.y = p.y();
        pose.position.z = p.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        return pose;
    };

    auto bezier_point = [](const Eigen::Vector3d& P0,
                           const Eigen::Vector3d& P1,
                           const Eigen::Vector3d& P2,
                           const Eigen::Vector3d& P3,
                           double s) {
        const double u = 1.0 - s;
        return (u * u * u) * P0 +
               (3.0 * u * u * s) * P1 +
               (3.0 * u * s * s) * P2 +
               (s * s * s) * P3;
    };

    auto clamp = [](double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    };

    RCLCPP_INFO(logger, "Ready for handover!!");
    until_object_ready(shared_pose, logger);

    // Curve progression variable.
    // 0 -> start pose, 1 -> object pose
    double s = 0.0;

    // Tune these
    constexpr double terminal_x_offset = 0.12;   // arrive from negative x side
    constexpr double start_pull = 0.25;          // P1 fraction from start toward goal
    constexpr double nominal_path_speed = 0.20;  // progress in path parameterized by geometric length
    constexpr double final_tol = 0.04;
    constexpr double min_progress_step = 0.003;
    constexpr double max_progress_step = 0.03;
    constexpr double dt = 0.05; // 50 ms loop

    while (rclcpp::ok()) {
        Eigen::Isometry3d target_tf;
        {
            std::lock_guard<std::mutex> lock(shared_pose->mtx);
            if (!shared_pose->pose.has_value()) {
                RCLCPP_ERROR(logger, "Lost grasp pose");
                res->success = false;
                res->message = "Lost grasp pose";
                return;
            }
            target_tf = *shared_pose->pose;
        }

        const geometry_msgs::msg::Pose target_pose = pose_from_isometry(target_tf);
        current_pose = *single_arm_control_interface->get_current_ee_pose();

        const double final_err = get_position_error(current_pose, target_pose).norm();
        if (final_err <= final_tol) {
            RCLCPP_INFO(logger, "Reached final grasp pose");
            break;
        }

        // Fixed anchor: original pose when tracking started
        const Eigen::Vector3d P0 = vector_from_pose(start_pose);
        const Eigen::Vector3d P3 = vector_from_pose(target_pose);

        // Path shaping:
        // P2 is behind target along -x, causing arrival along +x.
        Eigen::Vector3d P2 = P3 - Eigen::Vector3d{terminal_x_offset, 0.0, 0.0};

        // P1 pulls the curve out from start in a smooth manner.
        // This can be changed if you want a stronger initial shape.
        Eigen::Vector3d P1 = P0 + start_pull * (P2 - P0);

        // Interpolate position on spline
        Eigen::Vector3d path_pos = bezier_point(P0, P1, P2, P3, s);

        // Interpolate orientation from start to target
        Eigen::Quaterniond q0 = quaternion_from_pose(start_pose);
        Eigen::Quaterniond q3 = quaternion_from_pose(target_pose);
        Eigen::Quaterniond q_interp = q0.slerp(s, q3);
        q_interp.normalize();

        geometry_msgs::msg::Pose setpoint_pose = make_pose(path_pos, q_interp);
        ee_pose_tracker_interface->set_target_pose_(setpoint_pose);

        // Progress s based on approximate path length so motion is smoother
        const double approx_path_length =
            (P1 - P0).norm() + (P2 - P1).norm() + (P3 - P2).norm();

        double ds = 0.0;
        if (approx_path_length > 1e-6) {
            ds = (nominal_path_speed * dt) / approx_path_length;
        }
        ds = clamp(ds, min_progress_step, max_progress_step);

        // Advance along the freshly regenerated path.
        // Since the target updates, s is retained but curve geometry is recomputed every cycle.
        s = clamp(s + ds, 0.0, 1.0);

        rate.sleep();
    }


    // gripper on here

    ee_pose_tracker_interface->stop_tracking_();
    ee_pose_tracker_interface->clear_target_pose_();
    ee_pose_tracker_interface->unprepare_tracker_();

    current_pose = *single_arm_control_interface->get_current_ee_pose();
    current_pose.position.z = 0.07;

    // go down 
    single_arm_control_interface->execute_waypoints_cubic({current_pose},{0.5},0.3,0.0);

    // gripper off here

    // go up a bit
    current_pose.position.z += 0.1;
    single_arm_control_interface->execute_waypoints_cubic({current_pose},{0.2},0.3,0.0);

    single_arm_control_interface->move_to_joint_positions(
        {-0.251881, -1.21652, -2.01470, -3.057, -2.2568, 0.076789});

    RCLCPP_INFO(logger, "Finished handover sequence");
    res->success = true;
    res->message = "Handover completed";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("human_robot_handover");
    auto parallel_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto wall_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    auto logger = node->get_logger();

    auto shared_pose = std::make_shared<SharedGraspPose>();

    auto single_arm_control_interface = std::make_shared<BareBonesMoveit>(node);
    auto ee_pose_tracker_interface = std::make_shared<PoseTracker>(node);

    auto apr_tag_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object0_filtered_pose",
        10,
        [shared_pose](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            sub_cb(msg, shared_pose);
        });

    auto grasp_pose_publisher =
        node->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp_pose", 10);

    auto grasp_pose_publisher_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [wall_clock, shared_pose, grasp_pose_publisher]() {
            grasp_pose_publisher_timer_cb(wall_clock, shared_pose, grasp_pose_publisher);
        });

    auto handover_server = node->create_service<std_srvs::srv::Trigger>(
        "~/handover",
        [node, single_arm_control_interface, ee_pose_tracker_interface, shared_pose](
            std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr res) {
            callback(node, res, single_arm_control_interface, ee_pose_tracker_interface, shared_pose);
        },
        rmw_qos_profile_services_default,
        parallel_cb_group);

    RCLCPP_INFO(logger, "The node should be ready");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}