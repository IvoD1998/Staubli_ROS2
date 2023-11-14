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
    
    io_interface = Node(
        package="staubli_val3_driver",
        executable="staubli_io_interface",
        parameters=[
            {"robot_ip_address": robot_ip}
        ],
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            io_interface,
        ]
    )