cmake_minimum_required(VERSION 3.5)
project(robot_middleware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# check build type
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  # message("${PROJECT_NAME}: No build type specified. Selecting 'Release' for best performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# compile-out debug messages in Release configuration
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  # message("${PROJECT_NAME}: Setting ROS console log level to INFO in 'Release' config (compile-out debug messages)")
  add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simple_message REQUIRED)
find_package(motion_control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(control_msgs)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
  simple_message
  motion_control_msgs
  moveit_core
  moveit_ros_planning
  control_toolbox
  control_msgs
  EIGEN3
)
set(THIS_PACKAGE_LIBRARIES
  ${rclcpp_LIBRARIES}
  # ${simple_message_LIBRARIES}
  # ${motion_control_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
set(THIS_PACKAGE_INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  # ${simple_message_INCLUDE_DIRS}
  # ${motion_control_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCES
  src/*.cpp
)

add_executable(robot_middleware 
  ${SOURCES}
)
target_link_libraries(robot_middleware
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(robot_middleware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(robot_middleware
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_DEPENDS}
)


ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(
  TARGETS
    robot_middleware
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
