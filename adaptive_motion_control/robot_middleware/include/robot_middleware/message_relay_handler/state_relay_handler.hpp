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

#include "robot_middleware/message_relay_handler/message_relay_handler.hpp"

#include "moveit/robot_state/robot_state.hpp"
#include "simple_message/messages/joint_feedback_message.hpp"
#include "simple_message/messages/joint_message.hpp"
#include "simple_message/messages/robot_status_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/typed_message.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace robot_middleware
{
namespace message_relay_handler
{

class StateRelayHandler : public MessageRelayHandler
{
public:
  StateRelayHandler(std::shared_ptr<rclcpp::Node> node);

  ~StateRelayHandler();

  void init(const std::shared_ptr<RobotDriver>& driver,
            const robot_middleware::connection_manager::SimpleSocketManagerPtr& in,
            const robot_middleware::connection_manager::SimpleSocketManagerPtr& out) override;

protected:
  bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp) override;

  void handleMessage(industrial::joint_message::JointMessage& joint_msg, rclcpp::Time& timestamp);

  void handleMessage(industrial::joint_feedback_message::JointFeedbackMessage& joint_fbk_msg, rclcpp::Time& timestamp);

  void handleMessage(industrial::robot_status_message::RobotStatusMessage& robot_status_msg, rclcpp::Time& timestamp);

  double state_receive_timeout;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware