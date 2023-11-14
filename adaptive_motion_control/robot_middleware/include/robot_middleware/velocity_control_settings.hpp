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

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace robot_middleware
{

double toRadians(double angle_deg);

struct CartesianLimits
{

  double max_linear_velocity;
  double max_angular_velocity;
};

struct JointLimits
{
  double toRadians(double angle_deg);

  bool has_velocity_limit;
  double max_velocity;
  double stop_tolerance;
};

struct VelocityControlSettings
{
  explicit VelocityControlSettings(std::shared_ptr<rclcpp::Node> node);
  ~VelocityControlSettings();

  bool initParam();

  std::shared_ptr<rclcpp::Node> node_;
  std::string base_frame;
  std::string tool_frame;
  double control_loop_frequency;
  CartesianLimits cartesian_limits;
  JointLimits joint_limits;

private:
  const std::string LOGNAME;
};

}  // namespace robot_middleware