# preaction server
# cameras
# apriltag grid detector
# handover node
import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    tool0_x = "0.0"
    tool0_y = "0.0"
    tool0_z = "0.0"
    launch_cameras = LaunchConfiguration("launch_cameras")

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
            "tool0_x:=",
            tool0_x,
            " ",
            "tool0_y:=",
            tool0_y,
            " ",
            "tool0_z:=",
            tool0_z,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="preaction_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "ur_manipulator",
                "shoulder_pan": -0.2518866697894495,
                "shoulder_lift": -1.2164602738669892,
                "elbow": -2.014770746231079,
                "wrist_1": -3.0570813618102015,
                "wrist_2": -2.2569201628314417,
                "wrist_3": 0.0768733024597168,
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    right_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('realsense2_camera').find('realsense2_camera'), 
                'launch', 
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'align_depth.enable': 'true',
            'serial_no': '_211122061649',
            'pointcloud.enable': 'false',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
            'rgb_camera.color_profile':'640,480,30',
            'depth_module.color_profile':'640,480,30',
        }.items(),
        condition = IfCondition(launch_cameras),
    )

    right_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('corsmal_hrh_2026').find('corsmal_hrh_2026'), 
                'launch', 
                'right_camera_world_calibration.launch.py'
            )
        ),
    )

    left_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('realsense2_camera').find('realsense2_camera'), 
                'launch', 
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'align_depth.enable': 'true',
            'serial_no': '_210622060509',
            'pointcloud.enable': 'false',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
            'rgb_camera.color_profile':'640,480,30',
            'depth_module.color_profile':'640,480,30',
        }.items(),
        condition= IfCondition(launch_cameras),
    )

    left_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('corsmal_hrh_2026').find('corsmal_hrh_2026'), 
                'launch', 
                'left_camera_world_calibration.launch.py'
            )
        ),
    )

    human_robot_handover = Node(
        package="corsmal_hrh_2026",
        executable="human_robot_handover",
        name="human_robot_handover",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "planning_group": "ur_manipulator",
                "endeffector_link": "tool0",
                "tscubic_gen_traj_ns":"/task_space_cubic_polynomial_trajectory_server/generate_trajectory",
                "tscubic_exec_traj_ns":"/task_space_cubic_polynomial_trajectory_server/execute_trajectory",
                "set_io_ns":"/io_and_status_controller/set_io",
                "servo_node_ns": "/servo_node",
                "joint_traj_controller": "scaled_joint_trajectory_controller",
                "joint_vel_controller": "forward_velocity_controller",
                "alpha":0.8,
                "linear_P":1.0,
                "linear_D":0.0,
                "angular_P":2.0,
                "angular_D":0.0,
                "max_velocity":1.0,
            }
        ]
    )

    tsctge_node = Node(
        package="motion_planning_abstractions",
        executable="task_space_cubic_polynomial_trajectory_server",
        name="task_space_cubic_polynomial_trajectory_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group" : "ur_manipulator",
                "maximum_task_space_velocity" : 1.0,
                "maximum_task_space_acceleration" : 3.0,
                "maximum_joint_space_velocity" : 3.15,
                "maximum_joint_space_acceleration" : 3.14,
                "joint_prefix" : "",
                "joint_trajectory_controller" : "scaled_joint_trajectory_controller",
                "endeffector_link" : "tool0",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    apriltag_grid_detector = Node(
        package="apriltag_grid_detector",
        executable="apriltag_grid_detector",
        name="apriltag_grid_detector",
        output="screen",
        parameters=[
            {
                "alpha": 0.25,
                "marker_separation": 4.7,  # mm
                "marker_size": 27.84,        # mm
                "object.name": "object0",
                # 2 rows x 2 cols, flattened [row0, row1, ...]
                "grid.rows": 2,
                "grid.cols": 2,
                "grid.ids": [9, 10, 11, 12],
                "color_image_topic": "/camera/right_camera/color/image_raw",
                "camera_info_topic": "/camera/right_camera/color/camera_info",
                "depth_image_topic": "/camera/right_camera/depth/image_rect_raw",
                "detection_rate": 30.0,
            }
        ],
    )

    nodes_to_start = [
        # preaction_server,
        tsctge_node,
        # right_camera_launch,
        # left_camera_launch,
        # apriltag_grid_detector,
        # left_camera_calibration_launch,
        # right_camera_calibration_launch,
        human_robot_handover,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="single_arm_workcell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="single_arm_workcell_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_cameras",
            default_value="true",
            description="launch cameras?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])