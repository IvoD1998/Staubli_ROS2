cmake_minimum_required(VERSION 3.5)
project(simple_message)

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
find_package(industrial_msgs REQUIRED)

include_directories(
  include
  include/simple_message
  include/simple_message/messages
  include/simple_message/socket
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  industrial_msgs
)

set(SRC_FILES
  src/byte_array.cpp
	src/simple_message.cpp
	src/smpl_msg_connection.cpp

	src/socket/simple_socket.cpp
	src/socket/tcp_client.cpp
	src/socket/tcp_server.cpp
	src/socket/tcp_socket.cpp
	src/socket/udp_client.cpp
	src/socket/udp_server.cpp
	src/socket/udp_socket.cpp

	src/joint_data.cpp
	src/joint_feedback.cpp
	src/joint_traj_pt_full.cpp
	src/joint_traj_pt.cpp
	src/joint_traj.cpp
	src/message_handler.cpp
	src/message_manager.cpp
	src/ping_handler.cpp
	src/ping_message.cpp
	src/robot_status.cpp
  src/velocity_command.cpp
  src/velocity_config.cpp

	src/messages/joint_feedback_message.cpp
	src/messages/joint_message.cpp
	src/messages/joint_traj_pt_full_message.cpp
	src/messages/joint_traj_pt_message.cpp
	src/messages/robot_status_message.cpp
  src/messages/velocity_command_message.cpp
  src/messages/velocity_config_message.cpp

	src/simple_comms_fault_handler.cpp
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)
add_library(${PROJECT_NAME}_dummy
${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)

add_library(${PROJECT_NAME} 
  SHARED
  ${SRC_FILES})
target_compile_definitions(${PROJECT_NAME} PUBLIC
  SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX)
target_link_libraries(${PROJECT_NAME} $<$<BOOL:${WIN32}>:Ws2_32.lib>)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${industrial_msgs_INCLUDE_DIRS})

# add_library(${PROJECT_NAME}_bswap 
#   SHARED
#   ${SRC_FILES})
# NOTE: keep these in-sync with the lists in 'cmake/platform_build_flags.cmake'
# target_compile_definitions(${PROJECT_NAME}_bswap PUBLIC
#   SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX BYTE_SWAPPING)
# target_link_libraries(${PROJECT_NAME}_bswap $<$<BOOL:${WIN32}>:Ws2_32.lib>)
# ament_target_dependencies(${PROJECT_NAME}_bswap
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
# ament_export_targets(export_${PROJECT_NAME}_bswap HAS_LIBRARY_TARGET)
# target_include_directories(${PROJECT_NAME}_bswap
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#     ${rclcpp_INCLUDE_DIRS}
#     ${industrial_msgs_INCLUDE_DIRS})

# add_library(${PROJECT_NAME}_float64 
#   SHARED
#   ${SRC_FILES})
# # NOTE: keep these in-sync with the lists in 'cmake/platform_build_flags.cmake'
# target_compile_definitions(${PROJECT_NAME}_float64 PUBLIC
#   SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX FLOAT64)
# target_link_libraries(${PROJECT_NAME}_float64 $<$<BOOL:${WIN32}>:Ws2_32.lib>)
# ament_target_dependencies(${PROJECT_NAME}_float64
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
# ament_export_targets(export_${PROJECT_NAME}_float64 HAS_LIBRARY_TARGET)
# target_include_directories(${PROJECT_NAME}_float64
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#     ${rclcpp_INCLUDE_DIRS}
#     ${industrial_msgs_INCLUDE_DIRS})

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY cmake include src
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# install(
#   TARGETS ${PROJECT_NAME}_bswap
#   EXPORT export_${PROJECT_NAME}_bswap
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )
# install(
#   TARGETS ${PROJECT_NAME}_float64
#   EXPORT export_${PROJECT_NAME}_float64
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



