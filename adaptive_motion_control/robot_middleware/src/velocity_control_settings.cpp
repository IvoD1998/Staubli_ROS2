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

#include "robot_middleware/velocity_control_settings.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_middleware
{

double toRadians(double angle_degrees)
{
  double angle_rad = angle_degrees * (M_PI/180);
  return angle_rad;
}

VelocityControlSettings::VelocityControlSettings(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
}

VelocityControlSettings::~VelocityControlSettings()
{
}

bool VelocityControlSettings::initParam()
{
  // std::size_t error = 0;

  node_->declare_parameter<std::string>("velocity_control_base_frame", "base");
  node_->declare_parameter<std::string>("velocity_control_tool_frame", "tool0");
  node_->declare_parameter<double>("velocity_control_control_loop_frequency", 250.0);
  node_->declare_parameter<double>("velocity_control_max_linear_velocity", 0.250);
  node_->declare_parameter<double>("velocity_control_max_angular_velocity", 60.0);
  node_->declare_parameter<bool>("velocity_control_has_velocity_limit", true);
  node_->declare_parameter<double>("velocity_control_max_velocity", 60.0);
  node_->declare_parameter<double>("velocity_control_stop_tolerance", 3.0);

  node_->get_parameter<std::string>("velocity_control_base_frame", base_frame);
  node_->get_parameter<std::string>("velocity_control_tool_frame", tool_frame);
  node_->get_parameter<double>("velocity_control_control_loop_frequency", control_loop_frequency);
  node_->get_parameter<double>("velocity_control_max_linear_velocity", cartesian_limits.max_linear_velocity);
  node_->get_parameter<double>("velocity_control_max_angular_velocity", cartesian_limits.max_angular_velocity);
  node_->get_parameter<bool>("velocity_control_has_velocity_limit", joint_limits.has_velocity_limit);
  node_->get_parameter<double>("velocity_control_max_velocity", joint_limits.max_velocity);
  node_->get_parameter<double>("velocity_control_stop_tolerance", joint_limits.stop_tolerance);

  cartesian_limits.max_angular_velocity = toRadians(cartesian_limits.max_angular_velocity);
  joint_limits.max_velocity = toRadians(joint_limits.max_velocity);
  joint_limits.stop_tolerance = toRadians(joint_limits.stop_tolerance);

  return true;
}

}  // namespace robot_middleware
