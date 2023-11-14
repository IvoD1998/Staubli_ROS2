from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the robot",
    )
    robot_ip = LaunchConfiguration(
        "robot_ip",
    )

    robot_state_node = Node(
        package="staubli_val3_driver",
        executable="staubli_robot_state",
        parameters=[
            {"robot_ip_address": robot_ip}
        ],
        output="log",
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            robot_state_node,
        ]
    )