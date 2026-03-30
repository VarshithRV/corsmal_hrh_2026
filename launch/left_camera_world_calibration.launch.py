""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: left_wrist_3_link -> left_camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "world",
                "--child-frame-id",
                "left_camera_link",
                "--x",
                "-0.0164218",
                "--y",
                "-0.103843",
                "--z",
                "0.0241299",
                "--qx",
                "-0.506725",
                "--qy",
                "0.5087",
                "--qz",
                "-0.492059",
                "--qw",
                "-0.492273",
                # "--roll",
                # "1.60189",
                # "--pitch",
                # "-0.00216088",
                # "--yaw",
                # "1.57259",
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]
    return LaunchDescription(declared_arguments + nodes)
