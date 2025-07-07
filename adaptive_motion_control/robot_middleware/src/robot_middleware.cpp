/*
 * Copyright 2021 Institute for Factory Automation and Production Systems (FAPS)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include "robot_middleware/robot_middleware.hpp"

using namespace std::chrono_literals;

namespace robot_middleware
{

RobotMiddleware::RobotMiddleware() : Node("robot_middleware")
{
}

RobotMiddleware::~RobotMiddleware()
{
}

bool RobotMiddleware::init()
{
  jog_interface_ = std::make_shared<JogInterface>(this->shared_from_this());
  pose_tracking_controller_ = std::make_shared<PoseTrackingController>(this->shared_from_this());
  server_proxy_ = std::make_shared<RobotServerProxy>(this->shared_from_this());
  state_relay_handler_ = std::make_shared<robot_middleware::message_relay_handler::StateRelayHandler>(this->shared_from_this());
  motion_relay_handler_ = std::make_shared<robot_middleware::message_relay_handler::MotionRelayHandler>(this->shared_from_this());

  //We need to get the robot description as a parameter of this node to initialise the robot_model_loader properly.
  std::vector <rclcpp::Parameter> xml_strings = {};
  rclcpp::Parameter xml_string;
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  xml_strings = parameters_client->get_parameters({"robot_description"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("~robot_description", xml_string.as_string());

  //Do the same for the semantic description
  xml_strings = {};
  parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  xml_strings = parameters_client->get_parameters({"robot_description_semantic"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("~robot_description_semantic", xml_string.as_string());
  
  //Init the robot_model_loader
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "~robot_description", false);

  // get the robot IP from private parameter
  std::string robot_ip;

  this->declare_parameter<std::string>("robot_ip", "");
  this->get_parameter<std::string>("robot_ip", robot_ip);
  
  if (robot_ip == "")
  {
    RCLCPP_ERROR(this->get_logger(), "robot_ip parameter found empty!");
    return false;
  }
  // check if driver type is given as a parameter
  std::string driver_type = "staubli";
  driver_ = std::make_shared<staubli::StaubliDriver>(robot_ip, robot_model_loader_->getModel(), this->shared_from_this());

  // init the driver
  if (!driver_->init())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize the robot driver");
    return false;
  }

  // init the robot server proxy
  if (!server_proxy_->init(driver_->getDefaultMotionPort(), driver_->getDefaultStatePort()))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize the robot server proxy");
    return false;
  }

  // init the relay handlers
  state_relay_handler_->init(/* driver:  */ driver_,
                            /* in:      */ driver_->getStateClientManager(),
                            /* out:     */ server_proxy_->getStateServerManager());
  motion_relay_handler_->init(/* driver:  */ driver_,
                             /* in:      */ server_proxy_->getMotionServerManager(),
                             /* out:     */ driver_->getMotionClientManager());

  // init the controllers
  jog_interface_->init(driver_);
  pose_tracking_controller_->init(driver_);

  RCLCPP_INFO(this->get_logger(), "Initialized the middleware");

  return true;
}

void RobotMiddleware::run()
{

  // start message relay handlers
  motion_relay_handler_->start();
  state_relay_handler_->start();

  // start controllers
  jog_interface_->start();
  pose_tracking_controller_->start();

  RCLCPP_INFO(this->get_logger(), "All interfaces are ready and running. Starting the connection tasks.");

  // start async connection tasks
  driver_->connect();
  server_proxy_->connect();
}

}  // namespace robot_middleware
