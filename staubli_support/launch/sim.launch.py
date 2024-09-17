from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch required components
    launch_files = []
    nodes = []

    # Simulation only
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_tx2_60l_moveit_config"),
                             "launch", "staubli_tx2_60l_planning_execution_sim.launch.py")
            )
        )
    )
    return LaunchDescription(
        launch_files +
        nodes     
    )