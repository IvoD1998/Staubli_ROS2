from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

parameters_file_path = Path(get_package_share_directory('robot_middleware'), 'config', 'settings.yaml')

def generate_launch_description():

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
