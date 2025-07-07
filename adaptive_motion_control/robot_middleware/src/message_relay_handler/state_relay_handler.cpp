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

#include "robot_middleware/message_relay_handler/state_relay_handler.hpp"

#include <algorithm>
#include <array>

using namespace industrial::joint_data;
using namespace industrial::joint_feedback_message;
using namespace industrial::joint_message;
using namespace industrial::robot_status_message;
using namespace industrial::simple_message;
using namespace robot_middleware::connection_manager;

namespace robot_middleware
{
namespace message_relay_handler
{

StateRelayHandler::StateRelayHandler(std::shared_ptr<rclcpp::Node> node) : MessageRelayHandler("state_relay_handler", node)
{
  double state_receive_timeout_seconds = 0.0;
  node_->declare_parameter<double>("state_relay_handler_receive_timeout", 1.0);
  node_->get_parameter<double>("state_relay_handler_receive_timeout", state_receive_timeout_seconds);
  state_receive_timeout = state_receive_timeout_seconds * 1e3;
  RCLCPP_DEBUG(node_->get_logger(), "Initialized %s 'state_receive_timeout': %f ms", getName(), state_receive_timeout);

  receive_timeout_ = state_receive_timeout;
}

StateRelayHandler::~StateRelayHandler()
{
}

void StateRelayHandler::init(const std::shared_ptr<RobotDriver>& driver, const SimpleSocketManagerPtr& in,
                             const SimpleSocketManagerPtr& out)
{
  MessageRelayHandler::init(driver, in, out);
}

bool StateRelayHandler::handleMessage(SimpleMessage& msg, rclcpp::Time& timestamp)
{
  switch (msg.getMessageType())
  {
    case StandardMsgType::JOINT_POSITION:
    {
      JointMessage jnt_msg;
      if (jnt_msg.init(msg))
        handleMessage(jnt_msg, timestamp);
      break;
    }
    case StandardMsgType::JOINT_FEEDBACK:
    {
      JointFeedbackMessage jnt_fbk_msg;
      if (jnt_fbk_msg.init(msg))
        handleMessage(jnt_fbk_msg, timestamp);
      break;
    }
    case StandardMsgType::STATUS:
    {
      RobotStatusMessage status_msg;
      if (status_msg.init(msg))
        handleMessage(status_msg, timestamp);
      break;
    }
    default:
      RCLCPP_DEBUG(node_->get_logger(), "[%s] Received unknown message type: %d", getName(), msg.getMessageType());
      break;
  }

  // relay state messages by default
  return true;
}

void StateRelayHandler::handleMessage(JointMessage& joint_msg, rclcpp::Time& timestamp)
{
  const JointData& jnt_data = joint_msg.getJoints();
  std::array<double, RobotDriver::MAX_NUM_JOINTS> pos;
  int max_num_joints = std::min((int)RobotDriver::MAX_NUM_JOINTS, jnt_data.getMaxNumJoints());
  for (int i = 0; i < max_num_joints; ++i)
    pos[i] = jnt_data.getJoint(i);
  driver_->setJointPositions(pos, timestamp);
}

void StateRelayHandler::handleMessage(JointFeedbackMessage& jnt_fbk_msg, rclcpp::Time& timestamp)
{
  JointData jnt_pos_data, jnt_vel_data;
  std::array<double, RobotDriver::MAX_NUM_JOINTS> pos;
  std::array<double, RobotDriver::MAX_NUM_JOINTS> vel;
  bool has_position = jnt_fbk_msg.getPositions(jnt_pos_data);
  bool has_velocity = jnt_fbk_msg.getVelocities(jnt_vel_data);
  int max_num_joints = std::min((int)RobotDriver::MAX_NUM_JOINTS, jnt_pos_data.getMaxNumJoints());

  // at least position is required!
  if (has_position)
  {
    for (int i = 0; i < max_num_joints; ++i)
      pos[i] = jnt_pos_data.getJoint(i);

    if (has_velocity)
    {
      for (int i = 0; i < max_num_joints; ++i)
        vel[i] = jnt_vel_data.getJoint(i);

      driver_->setJointPositionsAndVelocities(pos, vel, timestamp);
    }
    else
    {
      driver_->setJointPositions(pos, timestamp);
    }

    // #ifndef NDEBUG
    //     std::string jnt_pos_str = std::to_string(pos[0]);
    //     for (int i = 1; i < pos.size(); ++i)
    //       jnt_pos_str += ", " + std::to_string(pos[i]);
    //     ROS_DEBUG_STREAM_NAMED(LOGNAME, "[" << name_ << "] Received joint feedback\n"
    //                                         << "  position: [" << jnt_pos_str << "]\n");
    // #endif
  }
}

void StateRelayHandler::handleMessage(RobotStatusMessage& robot_status_msg, rclcpp::Time& timestamp)
{
  driver_->setRobotStatus(robot_status_msg.status_, timestamp);
}

}  // namespace message_relay_handler
}  // namespace robot_middleware