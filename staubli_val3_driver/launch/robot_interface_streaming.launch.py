from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    #Declare all necessary parameters
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the robot",
        )
    )
    #Init arguments
    robot_ip = LaunchConfiguration(
        "robot_ip",
    )

    #Include all required launch files
    launch_files = []
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/robot_state.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items()
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/motion_streaming_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/io_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/system_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )

    #Nodes
    nodes = []
    nodes.append(
            Node(
            package="industrial_robot_client",
            executable="joint_trajectory_action",
        )
    )

    return LaunchDescription(
         declared_arguments + 
         launch_files +
         nodes
    )
