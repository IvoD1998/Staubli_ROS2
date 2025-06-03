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

#include "robot_middleware/robot_driver.hpp"
#include "robot_middleware/velocity_control_settings.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "motion_control_msgs/msg/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

enum class JogInterfaceState
{
  IDLE,
  CARTESIAN_JOGGING,
  JOINT_JOGGING
};

class JogInterface 
{
public:
  explicit JogInterface(std::shared_ptr<rclcpp::Node> node);

  ~JogInterface();

  void init(const std::shared_ptr<RobotDriver>& driver);

  void start();

  void stop();

  void reset();

private:
  void twistCb(const geometry_msgs::msg::TwistStamped::SharedPtr cmd);

  void jointJogCb(const control_msgs::msg::JointJog::SharedPtr cmd);

  bool inError(rclcpp::Time latest_cmd_timestamp);

  bool setCommand(const geometry_msgs::msg::TwistStamped::SharedPtr& cmd);

  bool setCommand(const control_msgs::msg::JointJog::SharedPtr& cmd);

  bool setState(JogInterfaceState new_state);

  void loop();

  bool controlLoop();

  const std::string LOGNAME = "jog_interface";
  const double DEFAULT_RECEIVE_TIMEOUT = 0.1;  // seconds

  std::shared_ptr<RobotDriver> driver_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr sub_joint_jog_;
  std::shared_ptr<rclcpp::Node> node_;
  geometry_msgs::msg::Twist twist_cmd_;
  control_msgs::msg::JointJog joint_jog_cmd_;
  motion_control_msgs::msg::VelocityCommand vel_cmd_;
  rclcpp::Time cmd_timestamp_;

  JogInterfaceState state_;
  double control_loop_hz_;
  double receive_timeout_;
  bool keep_running_;
  bool in_error_;

  std::thread control_task_;
  std::condition_variable command_received_cv_;
  std::mutex state_mtx_;
};

}  // namespace robot_middleware