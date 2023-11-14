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

#include "rclcpp/rclcpp.hpp"
#include "simple_message/messages/joint_traj_pt_full_message.hpp"
#include "simple_message/messages/joint_traj_pt_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/typed_message.hpp"

namespace robot_middleware
{
namespace message_relay_handler
{

class MotionRelayHandler : public MessageRelayHandler
{
public:
  MotionRelayHandler(std::shared_ptr<rclcpp::Node> node);

  ~MotionRelayHandler();

protected:
  void onReceiveTimeout() override;

  void onReceiveFail() override;

  bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp) override;

private:
  bool handleMessage(industrial::joint_traj_pt_message::JointTrajPtMessage& jnt_traj_pt_msg);

  bool handleMessage(industrial::joint_traj_pt_full_message::JointTrajPtFullMessage& jnt_traj_pt_full_msg);

  bool handleTrajectoryPoint(int sequence);

  bool is_relaying_;
  bool ignore_trajectory_;
  double motion_receive_timeout;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware