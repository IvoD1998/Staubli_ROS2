cmake_minimum_required(VERSION 3.5)
project(moveit_interface)

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
find_package(rclcpp REQUIRED)
find_package(motion_control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
  motion_control_msgs
  moveit_core
  moveit_ros_planning_interface
  std_srvs
)
set(THIS_PACKAGE_LIBRARIES
  ${rclcpp_LIBRARIES}
)
set(THIS_PACKAGE_INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
)

include_directories(
  include
)

add_executable(moveit_interface
  src/moveit_interface_node.cpp
  src/moveit_interface.cpp
)
target_link_libraries(moveit_interface
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(moveit_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(moveit_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_DEPENDS}
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(
  TARGETS
    moveit_interface
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
