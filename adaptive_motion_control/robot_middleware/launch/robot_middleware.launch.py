from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

parameters_file_path = Path(get_package_share_directory('robot_middleware'), 'config', 'settings.yaml')

def generate_launch_description():
     #Define launch arguments
    launch_arguments=[]
    #Declare arguments
    # launch_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_ip",
    #         description="IP address of the real robot"
    #     )
    # )
    # robot_ip = LaunchConfiguration(
    #     "robot_ip"
    # )
    # robot_ip = "172.31.0.2"
    #Nodes
    nodes = []
    nodes.append(
            Node(
            package="robot_middleware",
            executable="robot_middleware",
            output="screen",
            parameters=[
                parameters_file_path
            ],
        )
    )

    return LaunchDescription(
         nodes
    )
