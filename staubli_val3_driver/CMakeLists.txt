cmake_minimum_required(VERSION 3.5)
project(staubli_val3_driver)

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
find_package(control_msgs REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(industrial_robot_client REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_message REQUIRED)
find_package(staubli_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  include/staubli_val3_driver
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  control_msgs
  industrial_msgs
  industrial_robot_client
  sensor_msgs
  simple_message
  staubli_msgs
  rcpputils
  Boost
)
set(THIS_PACKAGE_INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${industrial_msgs_INCLUDE_DIRS}
  ${industrial_robot_client_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${simple_message_INCLUDE_DIRS}
  ${staubli_msgs_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
set(THIS_PACKAGE_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${industrial_msgs_LIBRARIES}
  ${industrial_robot_client_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${simple_message_LIBRARIES}
  ${staubli_msgs_LIBRARIES}
  ${rcpputils_LIBRARIES}
  ${Boost_LIBRARIES}
)

#Robot state node
add_executable(staubli_robot_state
  src/robot_state_node.cpp
  src/joint_feedback_relay_handler.cpp
)
target_link_libraries(staubli_robot_state
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(staubli_robot_state
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(staubli_robot_state
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#System interface node
add_executable(staubli_system_interface
  src/system_interface_node.cpp
  src/system_interface.cpp
  src/set_drive_power_message.cpp
)
target_link_libraries(staubli_system_interface
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(staubli_system_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(staubli_system_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#IO interface node
add_executable(staubli_io_interface
  src/io_interface_node.cpp
  src/io_interface.cpp
  src/io_states.cpp
  src/read_io_message.cpp
  src/write_single_io.cpp
  src/write_single_io_message.cpp
)
target_link_libraries(staubli_io_interface
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(staubli_io_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${THIS_PACKAGE_INCLUDE_DIRS} 
)
ament_target_dependencies(staubli_io_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS 
    staubli_robot_state
    staubli_io_interface
    staubli_system_interface
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALL
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

## EXPORTS
ament_export_include_directories(
  include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
