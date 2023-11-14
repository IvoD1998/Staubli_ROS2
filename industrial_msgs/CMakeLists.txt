cmake_minimum_required(VERSION 3.5)
project(industrial_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DebugLevel.msg"
  "msg/DeviceInfo.msg"
  "msg/RobotMode.msg"
  "msg/RobotStatus.msg"
  "msg/ServiceReturnCode.msg"
  "msg/TriState.msg"
  "srv/CmdJointTrajectory.srv"
  "srv/GetRobotInfo.srv"
  "srv/SetDrivePower.srv"
  "srv/SetRemoteLoggerLevel.srv"
  "srv/StartMotion.srv"
  "srv/StopMotion.srv"

  DEPENDENCIES std_msgs trajectory_msgs
)
ament_export_dependencies(rosidl_default_generators std_msgs trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
