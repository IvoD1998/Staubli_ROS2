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

#include "robot_middleware/jog_interface.hpp"

#include "tf2_eigen/tf2_eigen.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace std::placeholders;

namespace robot_middleware
{

JogInterface::JogInterface(std::shared_ptr<rclcpp::Node> node) : driver_(nullptr)
  , cmd_timestamp_(0)
  , state_(JogInterfaceState::IDLE)
  , control_loop_hz_(0)
  , keep_running_(false)
  , in_error_(false)
{
  node_ = node;
  node_->declare_parameter<double>("jog_interface_receive_timeout", DEFAULT_RECEIVE_TIMEOUT);
  node_->get_parameter<double>("jog_interface_receive_timeout", receive_timeout_);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized jog interface 'receive_timeout': %f", receive_timeout_);
}

JogInterface::~JogInterface()
{
  stop();
}

void JogInterface::init(const std::shared_ptr<RobotDriver>& driver)
{
  driver_ = driver;
  control_loop_hz_ = driver_->getVelocityControlSettings().control_loop_frequency;
  reset();
}

void JogInterface::start()
{
  keep_running_ = true;
  // rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(false)).best_effort().durability_volatile();
  sub_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("jog_interface/vel_cmd", 1, std::bind(&JogInterface::twistCb, this, _1));
  sub_joint_jog_ = node_->create_subscription<control_msgs::msg::JointJog>("jog_interface/joint_jog_cmd", 1, std::bind(&JogInterface::jointJogCb, this, _1));
  control_task_ = std::thread(&JogInterface::loop, this);
}

void JogInterface::stop()
{
  if (keep_running_)
  {
    // sub_twist_.shutdown();
    // sub_joint_jog_.shutdown();
    keep_running_ = false;
    command_received_cv_.notify_one();  // wake up thread if necessary
    if (control_task_.joinable())
      control_task_.join();
    reset();
  }
}

void JogInterface::reset()
{
  state_ = JogInterfaceState::IDLE;
  std::memset(vel_cmd_.cmd.data(), 0, sizeof(vel_cmd_.cmd));
  vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::UNDEFINED;
}

void JogInterface::twistCb(const geometry_msgs::msg::TwistStamped::SharedPtr cmd)
{
  rclcpp::Time timestamp = node_->now();
  std::lock_guard<std::mutex> lock(state_mtx_);
  bool in_error = inError(timestamp);
  cmd_timestamp_ = timestamp;

  if (!in_error && setState(JogInterfaceState::CARTESIAN_JOGGING))
  {
    if (!setCommand(cmd))
    {
      reset();
      return;
    }

    command_received_cv_.notify_one();
  }
}

void JogInterface::jointJogCb(const control_msgs::msg::JointJog::SharedPtr cmd) 
{
  rclcpp::Time timestamp = node_->now();
  std::lock_guard<std::mutex> lock(state_mtx_);
  bool in_error = inError(timestamp);
  cmd_timestamp_ = timestamp;

  if (!in_error && setState(JogInterfaceState::JOINT_JOGGING))
  {
    if (!setCommand(cmd))
    {
      reset();
      return;
    }

    command_received_cv_.notify_one();
  }
}

bool JogInterface::inError(rclcpp::Time latest_cmd_timestamp) 
{
  // check error state and recover after receive timeout
  if (in_error_ && (latest_cmd_timestamp - cmd_timestamp_).seconds() > receive_timeout_)
    in_error_ = false;

  if (in_error_)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                  "Ignoring current jog command stream when the jog interface was in error state. "
                  "Jogging will be available again after a receive timeout of %.2f seconds",
                  receive_timeout_);
  }

  return in_error_;
}

bool JogInterface::setCommand(const geometry_msgs::msg::TwistStamped::SharedPtr& cmd)
{
  // check frame id
  if (cmd->header.frame_id.empty())
  {
    // no error, just warn once
    RCLCPP_WARN_ONCE(node_->get_logger(),
        "No reference frame specified in twist command. Using the base frame '%s' as reference frame by default.",
        driver_->getVelocityControlSettings().base_frame.c_str());
    vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::BASE_FRAME;
  }
  else if (cmd->header.frame_id == driver_->getVelocityControlSettings().base_frame)
  {
    vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::BASE_FRAME;
  }
  else if (cmd->header.frame_id == driver_->getVelocityControlSettings().tool_frame)
  {
    vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::TOOL_FRAME;
  }
  else
  {
    auto robot_state = driver_->getRobotState();
    if (!robot_state.knowsFrameTransform(cmd->header.frame_id))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unknown frame '%s' in jog command", cmd->header.frame_id.c_str());

      return false;
    }

    auto base_link = robot_state.getLinkModel(driver_->getVelocityControlSettings().base_frame);
    auto tool_link = robot_state.getLinkModel(driver_->getVelocityControlSettings().tool_frame);
    auto reference_link = robot_state.getLinkModel(cmd->header.frame_id);

    // check for fixed transform to the base frame
    auto fixed_trsf_map_base = base_link->getAssociatedFixedTransforms();
    if (fixed_trsf_map_base.find(reference_link) != fixed_trsf_map_base.end())
    {
      RCLCPP_DEBUG(node_->get_logger(), 
                      "Reference frame '%s' has a fixed transform to the base frame. Transforming twist command.",
                      cmd->header.frame_id.c_str());

      // transform twist into the base frame
      Eigen::Isometry3d trsf = fixed_trsf_map_base[reference_link];
      Eigen::Matrix<double, 6, 1> twist_in;
      Eigen::Matrix<double, 6, 1> twist_out;
      tf2::fromMsg(cmd->twist, twist_in);
      twist_out.segment<3>(0) = trsf.linear() * twist_in.segment<3>(0);
      twist_out.segment<3>(3) = trsf.linear() * twist_in.segment<3>(3);

      // set the actual velocity command
      std::copy(twist_out.data(), twist_out.data() + twist_out.size(), vel_cmd_.cmd.data());
      vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::BASE_FRAME;

      return true;
    }

    // check for fixed transform to the tool frame
    auto fixed_trsf_map_tool = tool_link->getAssociatedFixedTransforms();
    if (fixed_trsf_map_tool.find(reference_link) != fixed_trsf_map_tool.end())
    {
      RCLCPP_DEBUG(node_->get_logger(),
                      "Reference frame '%s' has a fixed transform to the tool frame. Transforming twist command.",
                      cmd->header.frame_id.c_str());

      // transform twist into tool frame
      Eigen::Isometry3d tool_to_ref_trsf = fixed_trsf_map_tool[reference_link];
      Eigen::Matrix<double, 6, 1> twist_in;
      Eigen::Matrix<double, 6, 1> twist_out;
      tf2::fromMsg(cmd->twist, twist_in);
      twist_out.segment<3>(0) = tool_to_ref_trsf.linear() * twist_in.segment<3>(0);
      twist_out.segment<3>(3) = tool_to_ref_trsf.linear() * twist_in.segment<3>(3);

      // additional translation of the tool frame caused by rotation around the reference frame
      twist_out.segment<3>(0) += tool_to_ref_trsf.translation().cross(twist_out.segment<3>(3));

      // set the actual velocity command
      std::copy(twist_out.data(), twist_out.data() + twist_out.size(), vel_cmd_.cmd.data());
      vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::TOOL_FRAME;

      return true;
    }

    // error otherwise
    RCLCPP_ERROR(node_->get_logger(),
                             "The given reference frame '%s' can not be used for jogging. It should be the base frame "
                             "'%s', the tool frame '%s' or a fixed transform to one of these.",
                             cmd->header.frame_id.c_str(), driver_->getVelocityControlSettings().base_frame.c_str(),
                             driver_->getVelocityControlSettings().tool_frame.c_str());

    return false;
  }

  vel_cmd_.cmd[0] = cmd->twist.linear.x;
  vel_cmd_.cmd[1] = cmd->twist.linear.y;
  vel_cmd_.cmd[2] = cmd->twist.linear.z;
  vel_cmd_.cmd[3] = cmd->twist.angular.x;
  vel_cmd_.cmd[4] = cmd->twist.angular.y;
  vel_cmd_.cmd[5] = cmd->twist.angular.z;

  return true;
}

bool JogInterface::setCommand(const control_msgs::msg::JointJog::SharedPtr& cmd)
{
  vel_cmd_.type = motion_control_msgs::msg::VelocityCommand::JOINT;
  int max_axes = std::min(vel_cmd_.cmd.size(), cmd->velocities.size());
  for (int i = 0; i < max_axes; i++)
    vel_cmd_.cmd[i] = cmd->velocities[i];

  return true;
}

bool JogInterface::setState(JogInterfaceState new_state)
{
  if (state_ == new_state)
    return true;

  if (state_ == JogInterfaceState::IDLE)
    state_ = new_state;
  else
    RCLCPP_WARN(node_->get_logger(), "State transition (%d->%d) not allowed", (int)state_, (int)new_state);

  return (state_ == new_state);
}

void JogInterface::loop()
{
  RCLCPP_INFO(node_->get_logger(), "Ready to receive jog commands");

  while (rclcpp::ok() && keep_running_)
  {
    // sleep until new command received
    {
      std::unique_lock<std::mutex> lock(state_mtx_);
      reset();
      RCLCPP_INFO(node_->get_logger(), "Waiting for jog command...");
      command_received_cv_.wait(lock);

      // check if everything is ok!
      if (!rclcpp::ok() || !keep_running_)
        break;

      // wait again, if the state did not change
      if (state_ == JogInterfaceState::IDLE)
        continue;

      // scoped lock gets unlocked so the command data can be updated
    }

    // try to start velocity control and enter the control loop
    in_error_ = !driver_->startJogging(vel_cmd_.type);
    if (!in_error_)
    {
      in_error_ = !controlLoop();

      // stop
      std::memset(vel_cmd_.cmd.data(), 0, sizeof(vel_cmd_.cmd));
      in_error_ = !driver_->sendVelocityCommand(vel_cmd_);

      // set the driver in idle state
      driver_->stopMotionControl();
    }
  }
  RCLCPP_INFO(node_->get_logger(), "exiting loop");
}

bool JogInterface::controlLoop()
{
  motion_control_msgs::msg::VelocityCommand vel_cmd;
  RCLCPP_DEBUG(node_->get_logger(), "Entering the control loop");

  rclcpp::Time state_update_timestamp = node_->now();
  int state_wait_timeout = 100;  // in ms

  // ros::Rate rate(control_loop_hz_);
  while (rclcpp::ok() && keep_running_)
  {
    if (!driver_->waitForStateUpdate(state_update_timestamp, state_wait_timeout))
    {
      RCLCPP_WARN(node_->get_logger(), "No state update for %d ms", state_wait_timeout);
      return false;
    }

    {
      std::unique_lock<std::mutex> lock(state_mtx_);

      if (receive_timeout_ > 0 && (node_->now() - cmd_timestamp_).seconds() >= receive_timeout_)
      {
        RCLCPP_DEBUG(node_->get_logger(), "No jog command received until timeout. Exiting control loop.");
        break;
      }

      vel_cmd = vel_cmd_;

      // scoped lock gets unlocked so the command data can be updated
    }

    if (!driver_->sendVelocityCommand(vel_cmd))
      return false;

    // rate.sleep();
  }

  return true;
}

}  // namespace robot_middleware