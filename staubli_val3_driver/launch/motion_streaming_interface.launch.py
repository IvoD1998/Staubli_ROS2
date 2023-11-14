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
    
    motion_streaming_interface = Node(
        package="industrial_robot_client",
        executable="motion_streaming_interface",
        parameters=[
            {"robot_ip_address": robot_ip}
        ],
        output="log",
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            motion_streaming_interface,
        ]
    )