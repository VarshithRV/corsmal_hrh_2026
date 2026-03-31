#include <memory>
#include <functional>
#include <chrono>
#include <sstream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "motion_planning_abstractions/bare_bones_moveit.hpp"
#include "motion_planning_abstractions/ee_pose_tracker.hpp"
#include "rclcpp/executor.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#define HANDOVER_Z_THRESHOLD 0.3

void callback(
    rclcpp::Node::SharedPtr node, 
    std_srvs::srv::Trigger::Response::SharedPtr res,
    std::shared_ptr<BareBonesMoveit> single_arm_control_interface,
    std::shared_ptr<PoseTracker> ee_pose_tracker_interface,
    std::shared_ptr<Eigen::Isometry3d> current_grasp_pose_ptr
){
    auto logger = node->get_logger();
    
    RCLCPP_INFO(logger,"Initiating handover sequence");
    
    // preaction
    single_arm_control_interface->move_to_joint_positions({-0.251881,-1.21652,-2.01470,-3.057,-2.2568,0.076789});
    
    // write setpoint spline interpolation logic here
    if(current_grasp_pose_ptr==nullptr){
        RCLCPP_ERROR(logger,"No grasp pose received yet");
        res->success = false;
        return;
    }
    RCLCPP_INFO(logger,"Saw the object, now preparing the tracker");

    ee_pose_tracker_interface->prepare_tracker_();
    ee_pose_tracker_interface->clear_target_pose_();

    auto start_time = std::chrono::system_clock::now();
    auto run_duration = std::chrono::seconds(10);
    auto rate = rclcpp::Rate(std::chrono::milliseconds(50));
    geometry_msgs::msg::Pose start_pose = *single_arm_control_interface->get_current_ee_pose();
    geometry_msgs::msg::Pose current_setpoint_pose(start_pose),current_pose(start_pose);

    
    ee_pose_tracker_interface->set_target_pose_(*single_arm_control_interface->get_current_ee_pose());
    ee_pose_tracker_interface->start_tracking_();

    RCLCPP_INFO(logger,"Ready for handover!!");
    until_object_ready(current_grasp_pose_ptr);
    
    // modify current_setpoint and set that as target based while interpolating through "some" path between current_grasp_pose_ptr and start_pose
    // condition based on error between current_position

    // while(rclcpp::ok() && std::chrono::system_clock::now() - start_time < run_duration){
    //     ee_pose_tracker_interface->set_target_pose_(
    //         [single_arm_control_interface,start_time,start_pose](){
    //             start_pose->position.y += 0.001;
    //             return *start_pose;
    //         }()
    //     );
    //     rate.sleep();
    // }

    ee_pose_tracker_interface->stop_tracking_();
    ee_pose_tracker_interface->clear_target_pose_();
    ee_pose_tracker_interface->unprepare_tracker_();

    // preaction again
    single_arm_control_interface->move_to_joint_positions({-0.251881,-1.21652,-2.01470,-3.057,-2.2568,0.076789});


    RCLCPP_INFO(logger,"Finished handover sequence");
    res->success = true;
}

void until_object_ready(std::shared_ptr<Eigen::Isometry3d> current_grasp_pose_ptr){
    auto rate = rclcpp::Rate(10ms);
    while(current_grasp_pose_ptr->translation()[2] < HANDOVER_Z_THRESHOLD){
        rate.sleep();
    }
    return;
}

void sub_cb(
    rclcpp::Node::SharedPtr node, 
    geometry_msgs::msg::PoseStamped::SharedPtr msg, 
    std::shared_ptr<Eigen::Isometry3d> current_grasp_pose_ptr
){
    if(current_grasp_pose_ptr==nullptr){
        current_grasp_pose_ptr = std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity());
    }

    auto object_pose = Eigen::Isometry3d::Identity();
    object_pose.translate(Eigen::Vector3d{msg->pose.position.x,msg->pose.position.y,msg->pose.position.z});
    object_pose.rotate(Eigen::Quaterniond{msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z});

    auto dtransform = Eigen::Isometry3d::Identity();

    dtransform.translate(Eigen::Vector3d{0,0,0});
    dtransform.rotate(Eigen::Quaterniond{1,0,0,0});

    *current_grasp_pose_ptr = object_pose*dtransform;
}

void grasp_pose_publisher_timer_cb(
    std::shared_ptr<rclcpp::Clock> wall_clock, 
    std::shared_ptr<Eigen::Isometry3d> current_grasp_pose_ptr, 
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> publisher
){
    if(current_grasp_pose_ptr!=nullptr){
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.frame_id = "world";
        msg.header.stamp = wall_clock->now();
        msg.pose.position.x = current_grasp_pose_ptr->translation()[0];
        msg.pose.position.y = current_grasp_pose_ptr->translation()[1];
        msg.pose.position.z = current_grasp_pose_ptr->translation()[2];
        
        msg.pose.orientation.w = Eigen::Quaterniond(current_grasp_pose_ptr->rotation()).w();
        msg.pose.orientation.x = Eigen::Quaterniond(current_grasp_pose_ptr->rotation()).x();
        msg.pose.orientation.y = Eigen::Quaterniond(current_grasp_pose_ptr->rotation()).y();
        msg.pose.orientation.z = Eigen::Quaterniond(current_grasp_pose_ptr->rotation()).z();
        
        if(publisher != nullptr){
            publisher->publish(msg);
        }
    }
}

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<rclcpp::Node>("human_robot_handover");
    auto parallel_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto wall_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    auto logger = node->get_logger();

    // grasp pose ptr
    std::shared_ptr<Eigen::Isometry3d> current_grasp_pose_ptr;

    RCLCPP_INFO(logger,"Starting shit");

    // motion planning abstraction interfaces
    auto single_arm_control_interface = std::make_shared<BareBonesMoveit>(node);
    auto ee_pose_tracker_interface = std::make_shared<PoseTracker>(node);
    
    // subscription
    auto apr_tag_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object0_filtered_pose",
        10,
        [node,current_grasp_pose_ptr](geometry_msgs::msg::PoseStamped::SharedPtr msg){
            sub_cb(node, msg, current_grasp_pose_ptr);
        }
    );
    
    // pubs
    auto grasp_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/grasp_pose",
        10
    );

    // tims
    auto grasp_pose_publisher_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [wall_clock,current_grasp_pose_ptr,grasp_pose_publisher]{
            grasp_pose_publisher_timer_cb(wall_clock,current_grasp_pose_ptr,grasp_pose_publisher);
        }
    );

    // handover_cb
    auto handover_server = node->create_service<std_srvs::srv::Trigger>(
        "~/handover",
        [node,single_arm_control_interface,ee_pose_tracker_interface,current_grasp_pose_ptr](
            std_srvs::srv::Trigger::Request::SharedPtr, 
            std_srvs::srv::Trigger::Response::SharedPtr res
        ){
            callback(node,res,single_arm_control_interface,ee_pose_tracker_interface,current_grasp_pose_ptr);
        },
        rmw_qos_profile_services_default,
        parallel_cb_group
    );
    
    RCLCPP_INFO(logger,"The node should be ready");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor->add_node(node);
    executor->spin();
    
    rclcpp::shutdown();
}
