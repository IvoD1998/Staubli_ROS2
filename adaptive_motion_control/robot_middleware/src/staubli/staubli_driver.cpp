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

#include "robot_middleware/staubli/staubli_driver.hpp"

#include "simple_message/messages/velocity_command_message.hpp"
#include "simple_message/messages/velocity_config_message.hpp"

#include <algorithm>

using namespace industrial::shared_types;
using namespace industrial::simple_message;
using namespace moveit::core;

namespace staubli
{

StaubliDriver::StaubliDriver(const std::string& robot_ip, const RobotModelConstPtr& robot_model, std::shared_ptr<rclcpp::Node> node)
  : RobotDriver(robot_ip, robot_model, node)
{
}

StaubliDriver::~StaubliDriver()
{
}

bool StaubliDriver::sendVelocityCommand_internal(const motion_control_msgs::msg::VelocityCommand& vel_cmd)
{
  // convert command type
  industrial::velocity_command_type::VelocityCommandType vel_cmd_type = toStaubliVelocityCommandType(vel_cmd.type);
  if (vel_cmd_type == industrial::velocity_command_type::VelocityCommandType::INVALID)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert velocity command type (invalid)");
    return false;
  }

  industrial::velocity_command_message::VelocityCommandMessage vel_cmd_msg;
  vel_cmd_msg.data_.type_ = static_cast<shared_int>(vel_cmd_type);

  int max_num_joints = std::min(industrial::velocity_command::VelocityCommand::MAX_NUM_JOINTS, vel_cmd.cmd.size());
  for (int i = 0; i < max_num_joints; ++i)
    vel_cmd_msg.data_.vector_[i] = (shared_real)vel_cmd.cmd[i];

  SimpleMessage msg;
  vel_cmd_msg.toTopic(msg);

  return motion_client_manager_->sendMsg(msg);
}

bool StaubliDriver::sendVelocityConfig_internal(uint8_t type)
{
  industrial::velocity_config_message::VelocityConfigMessage vel_cfg_msg;
  industrial::velocity_config::VelocityConfig& cfg = vel_cfg_msg.data_;

  // convert command type
  industrial::velocity_command_type::VelocityCommandType vel_cmd_type = toStaubliVelocityCommandType(type);
  if (vel_cmd_type == industrial::velocity_command_type::VelocityCommandType::INVALID)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert velocity command type (invalid)");
    return false;
  }

  cfg.cmd_type_ = static_cast<shared_int>(vel_cmd_type);
  cfg.accel_ = 1.0;  // no acceleration limit (100% of nominal acceleration)

  // determine the maximum nominal velocity
  double max_nominal_velocity = 0.0;
  for (const VariableBounds& b : joint_limits_)
    max_nominal_velocity = std::max(max_nominal_velocity, b.max_velocity_);

  // determine the actual maximum velocity to use
  double max_actual_velocity = vel_ctrl_settings_->joint_limits.has_velocity_limit ?
                                   std::min(vel_ctrl_settings_->joint_limits.max_velocity, max_nominal_velocity) :
                                   max_nominal_velocity;

  cfg.vel_ = max_actual_velocity / max_nominal_velocity;  // determine % of nominal velocity for actual maximum velocity
  cfg.tvel_ = vel_ctrl_settings_->cartesian_limits.max_linear_velocity;
  cfg.rvel_ = vel_ctrl_settings_->cartesian_limits.max_angular_velocity;

  RCLCPP_DEBUG(node_->get_logger(), 
                  "Sending velocity config message\n"
                  "  cmd_type:  %d\n"
                  "  frame_ref: [%f, %f, %f, %f, %f, %f]\n"
                  "  tool_ref:  [%f, %f, %f, %f, %f, %f]\n"
                  "  accel:     %f\n"
                  "  vel:       %f\n"
                  "  tvel:      %f\n"
                  "  rvel:      %f\n",
                  cfg.cmd_type_, cfg.frame_ref_[0], cfg.frame_ref_[1], cfg.frame_ref_[2], cfg.frame_ref_[3],
                  cfg.frame_ref_[4], cfg.frame_ref_[5], cfg.tool_ref_[0], cfg.tool_ref_[1], cfg.tool_ref_[2],
                  cfg.tool_ref_[3], cfg.tool_ref_[4], cfg.tool_ref_[5], cfg.accel_, cfg.vel_, cfg.tvel_, cfg.rvel_);

  return sendRequest(vel_cfg_msg);
}

industrial::velocity_command_type::VelocityCommandType StaubliDriver::toStaubliVelocityCommandType(uint8_t type)
{
  auto rtn = industrial::velocity_command_type::VelocityCommandType::INVALID;

  switch (type)
  {
    case motion_control_msgs::msg::VelocityCommand::JOINT:
      rtn = industrial::velocity_command_type::VelocityCommandType::JOINT;
      break;

    case motion_control_msgs::msg::VelocityCommand::BASE_FRAME:
      rtn = industrial::velocity_command_type::VelocityCommandType::BASE_FRAME;
      break;

    case motion_control_msgs::msg::VelocityCommand::TOOL_FRAME:
      rtn = industrial::velocity_command_type::VelocityCommandType::TOOL_FRAME;
      break;

    default:
      break;
  }

  return rtn;
}

}  // namespace staubli
