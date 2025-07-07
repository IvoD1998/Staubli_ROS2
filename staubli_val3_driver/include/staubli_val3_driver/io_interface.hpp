
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

#pragma once

#include "write_single_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/socket/simple_socket.hpp"
#include "staubli_msgs/srv/write_single_io.hpp"
#include "netinet/in.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "stdio.h"


#include <string>

class IOInterface : public rclcpp::Node
{
public:
  static const std::map<int, std::string> MODULE_NAMES;

  IOInterface();

  ~IOInterface();

  bool init(const std::string& default_ip = "", int default_port = industrial::simple_socket::StandardSocketPort::IO);

  void run();

  void writeSingleIOCb(const std::shared_ptr<staubli_msgs::srv::WriteSingleIO::Request> req, 
                     std::shared_ptr<staubli_msgs::srv::WriteSingleIO::Response> res);

  bool writeSingleIO(IOModule moduleId, int pin, bool state);

private:
  std::shared_ptr<industrial::smpl_msg_connection::SmplMsgConnection> connection_;
  rclcpp::Service<staubli_msgs::srv::WriteSingleIO>::SharedPtr service;
};