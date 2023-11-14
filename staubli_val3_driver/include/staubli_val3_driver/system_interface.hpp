
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

#pragma once

#include <string>

#include "staubli_val3_driver/set_drive_power_message.hpp"
#include "industrial_msgs/srv/set_drive_power.hpp"
#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/smpl_msg_connection.hpp"

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface();

  ~SystemInterface();

  bool init(const std::string& default_ip = "",
            int default_port = industrial::simple_socket::StandardSocketPorts::SYSTEM);

  void run();
  
  void setDrivePowerCb(const std::shared_ptr<industrial_msgs::srv::SetDrivePower::Request> req,
                       std::shared_ptr<industrial_msgs::srv::SetDrivePower::Response> res);

  bool setDrivePower(bool value);

private:
  std::shared_ptr<industrial::smpl_msg_connection::SmplMsgConnection> connection_;
  rclcpp::Service<industrial_msgs::srv::SetDrivePower>::SharedPtr service;
};