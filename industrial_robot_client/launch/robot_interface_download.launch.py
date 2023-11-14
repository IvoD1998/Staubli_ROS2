from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip_address",
        description="IP address of the robot",
    )
    robot_ip = LaunchConfiguration(
        "robot_ip_address",
    )

    robot_state_node = Node(
        package="industrial_robot_client",
        executable="robot_state",
        parameters=[
            {"robot_ip_address": robot_ip}
        ],
    )
    motion_download_interface_node = Node(
        package="industrial_robot_client",
        executable="motion_download_interface",
    )
    joint_trajectory_action_node = Node(
        package="industrial_robot_client",
        executable="joint_trajectory_action",
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            robot_state_node,
            motion_download_interface_node,
            joint_trajectory_action_node,
        ]
    )