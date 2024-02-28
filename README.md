# Staubli_ROS2
ROS2 port of the Staubli_VAL3_ROS driver with all components it requires, expanded with FAU-FAPS adaptive_motion_middleware.

## general
This repository contains a port for ROS2 for everything that is required to use Staubli robots using the CS9 controller, based on the ROS1 driver (https://github.com/ros-industrial/staubli_val3_driver/tree/master).
This includes a port of the ROS simple message package and everything it requires (https://github.com/ros-industrial/industrial_core).
Also included is a port of FAU-FAPS adaptive_motion_middleware package to enable velocity and pose tracking control.
However, to use these features, you will need to install 'velocity' expansion for the CS9 controller.
For more details about this functionality, visit the original adaptive motion middleware repo at:
  https://github.com/FAU-FAPS/adaptive_motion_control.git
  
Included is a moveit config and description package for the Staubli TX2-60L model. If another CS9 capable robot is used, these files can be used as a template.
In the moveit config, there are two launch files:

Fully simulated robot: staubli_tx2_60l_planning_execution_sim.launch.py
  
Real robot: staubli_tx2_60l_planning_execution_real.launch.py

## Installing the VAL3 components
Copy the contents of the _staubli_val3_driver/val3_ folder to the CS9 controller via USB or an FTP client such as WinScp or the transfer manager found in the Staubli Robotics Suite.

## Configuring the robot
The TCP sockets on the robot controller must be set correctly prior to use. 
from the teach pendant home:
1) IO --> Socket --> TCP Servers --> "+"
2) Configure the following new sockets:
   
    | Name   | Port  | Timeout |End of string | Nagle |
    | ---    | ---   | ---     | ---          | ---   |
    | Motion | 11000 | -1      | 13           | Off   |
    | System | 11001 | -1      | 13           | Off   |
    | State  | 11002 | -1      | 13           | Off   |
    | IO     | 11003 | -1      | 13           | Off   |

## How to use
### Staubli-side:
Load the driver from the teach pendant home:
1) Application manager --> Val3 applications
2) +Disk --> ros_server
3) VAL# --> Memory --> select `ros_server` --> ▶

### ROS-side:
There are 3 different options for using this package:
  * Using a simulated robot:
    ```
    1) ros2 launch staubli_tx2_60l_planning_execution_sim.launch.py
    ```
  * Using a real robot with moveit only:
    ```
    1) ros2 launch staubli_tx2_60l_moveit_config staubli_tx2_60l_planning_execution_real.launch.py
    2) ros2 launch staubli_tx2_60l_moveit_config staubli_val3_driver robot_interface_streaming.launch.py robot_ip:=<ROBOT_IP>
    ```
  * Using a real robot with moveit and the adaptive motion middleware:
  
    (In this situation, some of the communication will go via the middleware, using IP 127.0.0.1)
    
    (For illustrative purposes, the entire launch sequence is written down here, in practice it is recommended to combine this in one launch file)
    ```
    1) ros2 launch robot_middleware robot_middleware.launch.py
    2) ros2 launch staubli_val3_driver robot_state.launch.py robot_ip:=127.0.1
    3) ros2 launch staubli_val3_driver motion_streaming_interface.launch.py robot_ip:=127.0.0.1
    4) ros2 launch staubli_val3_driver io_interface.launch.py robot_ip:=<ROBOT_IP>
    5) ros2 launch staubli_val3_driver system_interface.launch.py robot_ip:=<ROBOT_IP>
    6) ros2 launch staubli_tx2_60l_moveit_config staubli_tx2_60l_planning_execution_real.launch.py 
    7) ros2 run industrial_robot_client joint_trajectory_action
    8) ros2 run moveit_interface moveit_interface planning_group:=manipulator
    ```

  
 
