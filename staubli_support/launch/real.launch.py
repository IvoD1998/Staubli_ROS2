
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Define launch arguments
    launch_arguments=[]
    nodes = []
    launch_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the real robot",
        )
    )
    robot_ip = LaunchConfiguration(
        "robot_ip"
    )

    #Launch required components
    launch_files = []

    #Real robot, no middleware
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_tx2_60l_moveit_config"), "launch", "staubli_tx2_60l_planning_execution_real.launch.py")
            )
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_val3_driver"), "launch", "robot_interface_streaming.launch.py")
            ),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items()
        )
    )

    return LaunchDescription(
        launch_arguments +
        launch_files +
        nodes
    )
