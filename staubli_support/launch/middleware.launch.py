
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Define launch arguments
    launch_arguments=[]

    #Declare arguments
    launch_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the real robot"
        )
    )
    robot_ip = LaunchConfiguration(
        "robot_ip"
    )
    middleware_ip = "127.0.0.1"

    #Nodes
    nodes = []
    
    nodes.append(
        Node(
            package="moveit_interface",
            executable="moveit_interface",
            parameters=[
                {"planning_group": "manipulator"}
            ],
        )
    )   
    nodes.append(
            Node(
            package="industrial_robot_client",
            executable="joint_trajectory_action"
        )
    )

    #Launch required components
    launch_files = []

    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_val3_driver"), "launch", "io_interface.launch.py")
            ),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )
    
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_val3_driver"), "launch", "system_interface.launch.py")
            ),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items()
        )
    )
    
    # #Real robot, with middleware
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
                os.path.join(get_package_share_directory("robot_middleware"), "launch", "robot_middleware.launch.py")
            ),
            # launch_arguments={
            #     "robot_ip": robot_ip,
            # }.items()
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_val3_driver"), "launch", "robot_state.launch.py")
            ),
            launch_arguments={
                "robot_ip": middleware_ip,
            }.items()
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("staubli_val3_driver"), "launch", "motion_streaming_interface.launch.py")
            ),
            launch_arguments={
                "robot_ip": middleware_ip,
            }.items()
        )
    )
    

    return LaunchDescription(
        nodes +
        launch_arguments +
        launch_files 
    )
