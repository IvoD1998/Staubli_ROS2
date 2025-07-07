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

#pragma once

#include "robot_middleware/connection_manager/simple_socket_manager.hpp"
#include "robot_middleware/robot_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "simple_message/simple_message.hpp"

#include <string>
#include <thread>

namespace robot_middleware
{
namespace message_relay_handler
{

class MessageRelayHandler
{
public:
  explicit MessageRelayHandler(const std::string& name, std::shared_ptr<rclcpp::Node> node);

  ~MessageRelayHandler();

  virtual void init(const std::shared_ptr<RobotDriver>& driver,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& in,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& out);

  const char* getName();

  void spin();

  void spinOnce();

  void start();

  void stop();

protected:
  virtual void onReceiveTimeout();

  virtual void onReceiveFail();

  virtual bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp);

  bool sendReply(const robot_middleware::connection_manager::SimpleSocketManagerPtr& conn, int msg_type,
                 industrial::simple_message::ReplyType reply_type);

  const std::string LOGNAME = "message_relay_handler";
  const double DEFAULT_RECEIVE_TIMEOUT = 1.0;  // seconds

  const std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RobotDriver> driver_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr in_conn_mngr_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr out_conn_mngr_;
  int receive_timeout_;  // ms
  std::thread spinner_task_;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware