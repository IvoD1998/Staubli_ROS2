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

#include "robot_middleware/message_relay_handler/motion_relay_handler.hpp"

using namespace industrial::joint_traj_pt_message;
using namespace industrial::joint_traj_pt_full_message;
using namespace industrial::simple_message;

namespace robot_middleware
{
namespace message_relay_handler
{

MotionRelayHandler::MotionRelayHandler(std::shared_ptr<rclcpp::Node> node)
  : MessageRelayHandler("motion_relay_handler", node)
  , is_relaying_(false)
  , ignore_trajectory_(false)
  {
  double motion_receive_timeout_seconds = 0.0;
  node_->declare_parameter<double>("motion_relay_handler_receive_timeout", 0.1);
  node_->get_parameter<double>("motion_relay_handler_receive_timeout", motion_receive_timeout_seconds);
  motion_receive_timeout = motion_receive_timeout_seconds * 1e3;
  RCLCPP_DEBUG(node_->get_logger(), "Initialized %s 'motion_receive_timeout': %f ms", getName(), motion_receive_timeout);

  receive_timeout_ = motion_receive_timeout;
}

MotionRelayHandler::~MotionRelayHandler()
{
}

void MotionRelayHandler::onReceiveTimeout()
{
  if (is_relaying_)
  {
    driver_->setMotionControlMode(MotionControlMode::IDLE);
    is_relaying_ = false;
  }
}

void MotionRelayHandler::onReceiveFail()
{
  if (is_relaying_)
  {
    driver_->setMotionControlMode(MotionControlMode::IDLE);
    is_relaying_ = false;
  }
}

bool MotionRelayHandler::handleMessage(SimpleMessage& msg, rclcpp::Time& timestamp)
{
  // do not relay message by default
  is_relaying_ = false;

  MessageRelayHandler::handleMessage(msg, timestamp);

  switch (msg.getMessageType())
  {
    case StandardMsgType::JOINT_TRAJ_PT:
    {
      JointTrajPtMessage traj_pt_msg;
      if (traj_pt_msg.init(msg))
        is_relaying_ = handleMessage(traj_pt_msg);
      break;
    }
    case StandardMsgType::JOINT_TRAJ_PT_FULL:
    {
      JointTrajPtFullMessage traj_pt_full_msg;
      if (traj_pt_full_msg.init(msg))
        is_relaying_ = handleMessage(traj_pt_full_msg);
      break;
    }
    default:
      RCLCPP_INFO(node_->get_logger(), "[%s] Relaying unhandled message (msg_type: %d)", getName(), msg.getMessageType());
      is_relaying_ = driver_->setMotionControlMode(MotionControlMode::MOTION_COMMAND_RELAYING);
      break;
  }

  return is_relaying_;
}

bool MotionRelayHandler::handleMessage(JointTrajPtMessage& jnt_traj_pt_msg)
{
  auto& traj_pt = jnt_traj_pt_msg.point_;
  RCLCPP_DEBUG(node_->get_logger(), "[%s] Handling JointTrajPtMessage (sequence: %d)", getName(), traj_pt.getSequence());

  return handleTrajectoryPoint(traj_pt.getSequence());
}

bool MotionRelayHandler::handleMessage(JointTrajPtFullMessage& jnt_traj_pt_full_msg)
{
  auto& traj_pt = jnt_traj_pt_full_msg.point_;
  RCLCPP_INFO(node_->get_logger(), "[%s] Handling JointTrajPtFullMessage (sequence: %d)", getName(), traj_pt.getSequence());

  return handleTrajectoryPoint(traj_pt.getSequence());
}

bool MotionRelayHandler::handleTrajectoryPoint(int sequence)
{
  // ignore the rest of the trajectory if the first point can't be relayed
  if (sequence == 0)
    ignore_trajectory_ = !driver_->setMotionControlMode(MotionControlMode::JOINT_TRAJECTORY_STREAMING);

  return !ignore_trajectory_;
}

}  // namespace message_relay_handler
}  // namespace robot_middleware