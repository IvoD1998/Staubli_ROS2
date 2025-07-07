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

#include "robot_middleware/robot_driver.hpp"
#include "robot_middleware/velocity_control_settings.hpp"

#include "Eigen/Dense"
#include "angles/angles.h"
#include "control_toolbox/pid.hpp"
#include "motion_control_msgs/msg/pid_parameters.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_control_msgs/msg/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

class PoseTrackingController
{
public:
  PoseTrackingController(std::shared_ptr<rclcpp::Node> node);

  ~PoseTrackingController();

  void init(const std::shared_ptr<RobotDriver>& driver);

  void start();

  void stop();

  void reset();

private:
  bool initPid(const motion_control_msgs::msg::PIDParameters &pid_params, control_toolbox::Pid& pid, const std::string &name);

  void poseGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr goal);

  bool hasReachedGoal(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                      rclcpp::Time goal_timestamp);

  void computeCommand(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                      motion_control_msgs::msg::VelocityCommand& vel_cmd);

  void loop();

  double toRadians(double angle_deg);

  bool controlLoop();

  const std::string LOGNAME = "pose_tracking";
  const std::string DEFAULT_TCP_FRAME = "tool0";
  const double DEFAULT_GOAL_TOLERANCE_LINEAR = 0.0001;                      // 0.1 mm
  const double DEFAULT_GOAL_TOLERANCE_ANGULAR = angles::from_degrees(0.1);  // 0.1 deg
  const double DEFAULT_GOAL_SETTLE_TIME = 0.1;                              // 100 ms
  const double DEFAULT_RECEIVE_TIMEOUT = 1.0;                               // 1.0 second
  motion_control_msgs::msg::PIDParameters DEFAULT_PID_LINEAR;
  motion_control_msgs::msg::PIDParameters DEFAULT_PID_ANGULAR;

  std::shared_ptr<RobotDriver> driver_;

  rclcpp::Duration controller_period_;
  rclcpp::Time goal_timestamp_;
  rclcpp::Time last_goal_timestamp_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::shared_ptr<rclcpp::Node> node_;
  geometry_msgs::msg::PoseStamped goal_;

  control_toolbox::Pid pid_linear_x_;
  control_toolbox::Pid pid_linear_y_;
  control_toolbox::Pid pid_linear_z_;
  control_toolbox::Pid pid_angular_;

  std::string tcp_frame_;
  double goal_tolerance_linear_;
  double goal_tolerance_angular_;
  double goal_settle_time_;
  double receive_timeout_;
  bool keep_running_;
  bool in_error_;
  motion_control_msgs::msg::PIDParameters pid_linear_x_params_;
  motion_control_msgs::msg::PIDParameters pid_linear_y_params_;
  motion_control_msgs::msg::PIDParameters pid_linear_z_params_;
  motion_control_msgs::msg::PIDParameters pid_angular_params_;

  std::thread control_task_;
  std::condition_variable goal_received_;
  std::mutex goal_mtx_;
};

}  // namespace robot_middleware