
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

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control and https://github.com/ros-industrial/staubli_val3_driver/tree/master
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include <string>

#include "staubli_val3_driver/system_interface.hpp"
#include "staubli_val3_driver/set_drive_power_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "industrial_msgs/srv/set_drive_power.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/socket/tcp_client.hpp"

using namespace std::placeholders;

SystemInterface::SystemInterface() 
  : Node("system_interface")
  , connection_(nullptr)
{
}

SystemInterface::~SystemInterface()
{
}

bool SystemInterface::init(const std::string& default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->declare_parameter<int>("~port", default_port);
  this->get_parameter("robot_ip_address", ip);
  this->get_parameter("~port", port);

  // check for valid parameter values
  if (ip.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP address found. Please set the 'robot_ip_address' parameter");
    return false;
  }
  if (port <= 0 || port > 65535)
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP port found. Please set the '~port' parameter");
    return false;
  }

  // connection.init() requires "char*", not "const char*"
  char* ip_addr = strdup(ip.c_str());

  // create and connect client connection
  auto client = std::make_shared<industrial::tcp_client::TcpClient>();
  bool rtn = client->init(ip_addr, port);
  free(ip_addr);

  if (!rtn)
    return false;

  this->connection_ = client;
  RCLCPP_INFO(this->get_logger(), "system_interface: Connecting (%s:%d)", ip.c_str(), port);

  return this->connection_->makeConnect();
}

void SystemInterface::run()
{
  service = this->create_service<industrial_msgs::srv::SetDrivePower>("system_interface/set_drive_power", std::bind(&SystemInterface::setDrivePowerCb, this, _1, _2));
  RCLCPP_INFO_STREAM(this->get_logger(), "Service " << service->get_service_name() << " is ready and running");
}

void SystemInterface::setDrivePowerCb(const std::shared_ptr<industrial_msgs::srv::SetDrivePower::Request> req,
                                      std::shared_ptr<industrial_msgs::srv::SetDrivePower::Response> res)
{
  if (req->drive_power)
    RCLCPP_INFO(this->get_logger(), "Setting drive power ON");
  else
    RCLCPP_INFO(this->get_logger(), "Setting drive power OFF");

  if (setDrivePower(req->drive_power))
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::FAILURE;
  }
}

bool SystemInterface::setDrivePower(bool value)
{
  SetDrivePowerMessage msg;
  industrial::simple_message::SimpleMessage send, reply;
  msg.init(value);
  msg.toRequest(send);
  connection_->sendAndReceiveMsg(send, reply);
  return reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS;
}