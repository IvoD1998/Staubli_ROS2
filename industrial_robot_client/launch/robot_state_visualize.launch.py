from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro
import os


def generate_launch_description():

    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip_address",
        description="IP address of the robot",
    )
    robot_ip = LaunchConfiguration(
        "robot_ip_address",
    )
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("staubli_tx2_60l_description"),
            "urdf",
            "staubli_tx2_60l.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    rviz_config_file = PathJoinSubstitution([FindPackageShare("industrial_robot_client"), "rviz","rviz_config_empty.rviz"])


    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    robot_state_node = Node(
        package="industrial_robot_client",
        executable="robot_state",
        name="robot_state",
        parameters=[
            {"robot_ip_address": robot_ip}
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            joint_state_publisher_node,
            robot_state_publisher,
            robot_state_node,
            rviz_node
        ]
    )