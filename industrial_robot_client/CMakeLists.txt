cmake_minimum_required(VERSION 3.5)
project(industrial_robot_client)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_message REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(industrial_utils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  include/industrial_robot_client
)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  simple_message
  industrial_msgs
  industrial_utils
  rcpputils
  Boost
)
set(THIS_PACKAGE_INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${simple_message_INCLUDE_DIRS}
  ${industrial_msgs_INCLUDE_DIRS}
  ${industrial_utils_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
set(THIS_PACKAGE_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${simple_message_LIBRARIES}
  ${industrial_msgs_LIBRARIES}
  ${industrial_utils_LIBRARIES}
  ${rcpputils_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${Boost_LIBRARIES}
)
################################
######### LIBRARIES ############
################################
#Add code libraries in both regular and bswap form
add_library(${PROJECT_NAME}
  SHARED
  src/joint_relay_handler.cpp
  src/joint_trajectory_downloader.cpp
  src/joint_trajectory_interface.cpp
  src/joint_trajectory_streamer.cpp
  src/robot_state_interface.cpp
  src/robot_status_relay_handler.cpp
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# add_library(${PROJECT_NAME}_bswap
#   SHARED
#   src/joint_relay_handler.cpp
#   src/joint_trajectory_downloader.cpp
#   src/joint_trajectory_interface.cpp
#   src/joint_trajectory_streamer.cpp
#   src/robot_state_interface.cpp
#   src/robot_status_relay_handler.cpp
#   src/utils.cpp
# )
# target_link_libraries(${PROJECT_NAME}_bswap
#   ${THIS_PACKAGE_LIBRARIES}
# )
# target_include_directories(${PROJECT_NAME}_bswap
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>  
#   ${THIS_PACKAGE_INCLUDE_DIRS}
# )
# ament_target_dependencies(${PROJECT_NAME}_bswap
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

################################
########## Action #############
################################

add_library(action_server 
  SHARED
  src/joint_trajectory_action.cpp
)
target_link_libraries(action_server
  ${PROJECT_NAME}
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(action_server
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  # ${THIS_PACKAGE_INCLUDE_DIRS}
)
target_compile_definitions(action_server
  PRIVATE
  "INDUSTRIAL_ROBOT_CLIENT_BUILDING_DLL"
)
ament_target_dependencies(action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rclcpp_components_register_node(action_server PLUGIN "industrial_robot_client::joint_trajectory_action::JointTrajectoryAction" EXECUTABLE joint_trajectory_action)

################################
######## Executables ###########
################################
#Regular
#Robot state
add_executable(robot_state
  src/generic_robot_state_node.cpp
)
target_link_libraries(robot_state
  ${PROJECT_NAME}
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(robot_state
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(robot_state
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_definitions(robot_state PRIVATE
  ${simple_message_DEFINITIONS})
#Motion streaming
add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp
)
target_link_libraries(motion_streaming_interface
  ${PROJECT_NAME} 
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(motion_streaming_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(motion_streaming_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_definitions(motion_streaming_interface PRIVATE
  ${simple_message_DEFINITIONS})
#Motion downloading
add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp
)
target_link_libraries(motion_download_interface
  ${PROJECT_NAME}  
  ${THIS_PACKAGE_LIBRARIES}
)
target_include_directories(motion_download_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(motion_download_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_definitions(motion_download_interface PRIVATE
  ${simple_message_DEFINITIONS})

# #Bswap
# #Robot state
# add_executable(robot_state_bswap
#   src/generic_robot_state_node.cpp
# )
# target_link_libraries(robot_state_bswap
#   ${PROJECT_NAME}_bswap
#   ${THIS_PACKAGE_LIBRARIES}
# )
# target_include_directories(robot_state_bswap
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>  
#   ${THIS_PACKAGE_INCLUDE_DIRS}
# )
# ament_target_dependencies(robot_state_bswap
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
# target_compile_definitions(robot_state_bswap PRIVATE
#   ${simple_message_bswap_DEFINITIONS})
# #Motion streaming
# add_executable(motion_streaming_interface_bswap
#   src/generic_joint_streamer_node.cpp
# )
# target_link_libraries(motion_streaming_interface_bswap
#   ${PROJECT_NAME}_bswap 
#   ${THIS_PACKAGE_LIBRARIES}
# )
# target_include_directories(motion_streaming_interface_bswap
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>  
#   ${THIS_PACKAGE_INCLUDE_DIRS}
# )
# ament_target_dependencies(motion_streaming_interface_bswap
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
# target_compile_definitions(motion_streaming_interface_bswap PRIVATE
#   ${simple_message_bswap_DEFINITIONS})
# #Motion downloading
# add_executable(motion_download_interface_bswap
#   src/generic_joint_downloader_node.cpp
# )
# target_link_libraries(motion_download_interface_bswap
#   ${PROJECT_NAME}_bswap
#   ${THIS_PACKAGE_LIBRARIES}
# )
# target_include_directories(motion_download_interface_bswap
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>  
#   ${THIS_PACKAGE_INCLUDE_DIRS}
# )
# ament_target_dependencies(motion_download_interface_bswap
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
# target_compile_definitions(motion_download_interface_bswap PRIVATE
#   ${simple_message_bswap_DEFINITIONS})



ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

################################
########### Install ############
################################
#Libraries
install(
  TARGETS 
  ${PROJECT_NAME}
  # ${PROJECT_NAME}_bswap
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#Executables
install(
  TARGETS
    robot_state
    motion_streaming_interface
    motion_download_interface
    # robot_state_bswap
    # motion_streaming_interface_bswap
    # motion_download_interface_bswap
    # joint_trajectory_action
  DESTINATION lib/${PROJECT_NAME}
)

# Action
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
install(
  DIRECTORY config cmake launch rviz
  DESTINATION share/${PROJECT_NAME}
)

################################
########### Testing ############
################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
