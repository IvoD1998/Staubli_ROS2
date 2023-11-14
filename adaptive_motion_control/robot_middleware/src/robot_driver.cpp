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

#include "robot_middleware/robot_driver.hpp"

#include "robot_middleware/connection_manager/tcp_client_manager.hpp"

#include "angles/angles.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>

using namespace eigen_typedefs;
using namespace industrial::robot_status;
using namespace industrial::simple_message;
using namespace moveit::core;
using namespace robot_middleware::connection_manager;

namespace robot_middleware
{

const std::map<MotionControlMode, const char*> RobotDriver::MOTION_CTRL_MODE_NAMES = {
  { MotionControlMode::IDLE, "IDLE" },
  { MotionControlMode::LOCKED, "LOCKED" },
  { MotionControlMode::JOINT_TRAJECTORY_STREAMING, "JOINT_TRAJECTORY_STREAMING" },
  { MotionControlMode::MOTION_COMMAND_RELAYING, "MOTION_COMMAND_RELAYING" },
  { MotionControlMode::JOGGING, "JOGGING" },
  { MotionControlMode::POSE_TRACKING, "POSE_TRACKING" }
};

RobotDriver::RobotDriver(const std::string& robot_ip, const RobotModelConstPtr& robot_model, std::shared_ptr<rclcpp::Node> node) : robot_ip_(robot_ip)
  , motion_client_manager_(nullptr)
  , state_client_manager_(nullptr)
  , default_motion_port_(industrial::simple_socket::StandardSocketPort::MOTION)
  , default_state_port_(industrial::simple_socket::StandardSocketPort::STATE)
  , robot_state_(robot_model)
  , motion_ctrl_mode_(MotionControlMode::IDLE)
{
  node_ = node;
  initSetup();
}

RobotDriver::~RobotDriver()
{
}

void RobotDriver::initSetup()
{
  //Declare all parameters 
  node_->declare_parameter<std::string>("robot_driver_planning_group", "manipulator");
  node_->declare_parameter<bool>("robot_driver_publish_vel_cmd", true);

  //Get parameters
  node_->get_parameter<std::string>("robot_driver_planning_group", planning_group_);
  node_->get_parameter<bool>("robot_driver_publish_vel_cmd", publish_vel_cmd_);

  RCLCPP_INFO(node_->get_logger(), "Using planning_group: %s", planning_group_);

  // get joint model group
  joint_model_group_ = robot_state_.getJointModelGroup(planning_group_);
  if (!joint_model_group_)
    throw std::runtime_error("Failed to get joint model group '" + planning_group_ + "' from the robot model");

  num_joints_ = joint_model_group_->getActiveJointModels().size();

  // get the velocity control settings from parameter server
  vel_ctrl_settings_ = std::make_shared<VelocityControlSettings>(node_->shared_from_this());
  vel_ctrl_settings_->initParam();

  // check if base and tool frames specified in vel_ctrl_settings exist in the robot model
  if (!robot_state_.knowsFrameTransform(vel_ctrl_settings_->base_frame))
    throw std::runtime_error("Base frame '" + vel_ctrl_settings_->base_frame + "' not found in robot model");

  if (!robot_state_.knowsFrameTransform(vel_ctrl_settings_->tool_frame))
    throw std::runtime_error("Tool frame '" + vel_ctrl_settings_->tool_frame + "' not found in robot model");

  // init joint limits
  for (auto joint_model : joint_model_group_->getActiveJointModels())
  {
    // clang-format off
    if (joint_model->getType() != moveit::core::JointModel::REVOLUTE ||
        joint_model->getVariableCount() < 1 || joint_model->getVariableCount() > 1)
      throw std::runtime_error("Only revolute joints with one variable (single DOF) are supported\n"
                               "  group:     " + joint_model_group_->getName() + "\n"
                               "  joint:     " + joint_model->getName()        + "\n"
                               "  type:      " + joint_model->getTypeName()    + "\n"
                               "  variables: " + std::to_string(joint_model->getVariableCount()));
    // clang-format on

    joint_names_.push_back(joint_model->getName());
    joint_limits_.push_back(joint_model->getVariableBounds()[0]);
  }

  // populate joint limits map
  for (int i = 0; i < joint_names_.size(); ++i)
    joint_limits_map_.emplace(joint_names_[i], std::ref(joint_limits_[i]));

  // print joint names
  std::string names_str;
  for (const std::string& name : joint_names_)
    names_str += (names_str.empty() ? "" : ", ") + name;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Joint names: [" << names_str << "]");

  // print joint limits
  RCLCPP_DEBUG(node_->get_logger(), "Joint limits:");
  for (int i = 0; i < joint_names_.size(); ++i)
    RCLCPP_DEBUG_STREAM(node_->get_logger(), joint_names_[i] << ": " << joint_limits_[i]);

  // init publishers
  if (publish_vel_cmd_)
  {
    raw_vel_cmd_publisher_ = node_->create_publisher<motion_control_msgs::msg::VelocityCommand>("robot_driver/raw_vel_cmd", 10);
    filt_vel_cmd_publisher_ = node_->create_publisher<motion_control_msgs::msg::VelocityCommand>("robot_driver/filt_vel_cmd", 10);
  }

  // init robot state
  robot_state_.setToDefaultValues();
}

bool RobotDriver::init()
{
  int motion_port, state_port;
  node_->declare_parameter<int>("robot_driver_motion_port", default_motion_port_);
  node_->declare_parameter<int>("robot_driver_state_port", default_state_port_);
  node_->get_parameter<int>("robot_driver_motion_port", motion_port);
  node_->get_parameter<int>("robot_driver_state_port", state_port);

  return init(motion_port, state_port);
}

bool RobotDriver::init(int motion_port, int state_port)
{
  RCLCPP_INFO(node_->get_logger(), "Using motion port: %d", motion_port);
  RCLCPP_INFO(node_->get_logger(), "Using state port:  %d", state_port);

  auto motion_client_manager = std::make_shared<TcpClientManager>("motion_client", robot_ip_, motion_port);
  auto state_client_manager = std::make_shared<TcpClientManager>("state_client", robot_ip_, state_port);

  if (!motion_client_manager->init())
    return false;

  if (!state_client_manager->init())
    return false;

  motion_client_manager_ = motion_client_manager;
  state_client_manager_ = state_client_manager;
  state_client_manager_->setFriendConnection(motion_client_manager_);

  return true;
}

bool RobotDriver::connect()
{
  motion_client_manager_->startConnectionTask();
  state_client_manager_->startConnectionTask();

  return true;
}

bool RobotDriver::isConnected()
{
  return state_client_manager_->isConnected() && motion_client_manager_->isConnected();
}

bool RobotDriver::isMotionPossible()
{
  if ((node_->now() - status_timestamp_).seconds() > 1.0)
  {
    RCLCPP_WARN(node_->get_logger(), "Robot status message is older than 1.0 second. Assuming that motion is not possible");
    return false;
  }

  RobotStatus status = getRobotStatus();

  if (status.getMotionPossible() != TriState::TS_TRUE)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "RobotStatus.isMotionPossible: " << status.getMotionPossible());
    return false;
  }

  if (status.getDrivesPowered() != TriState::TS_TRUE)
  {
    RCLCPP_WARN(node_->get_logger(), "Drives not powered");
  }

  return true;
}

bool RobotDriver::getTransform(const std::string& tcp_frame, const std::string& reference_frame, Eigen::Isometry3d& trsf)
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  return getTransform_internal(tcp_frame, reference_frame, trsf);
}

bool RobotDriver::setMotionControlMode(MotionControlMode mode)
{
  std::lock_guard<std::mutex> lock(motion_ctrl_mtx_);

  MotionControlMode last_mode = motion_ctrl_mode_;
  std::thread::id caller_id = std::this_thread::get_id();

  if (last_mode == MotionControlMode::IDLE || motion_ctrl_owner_ == caller_id)
  {
    motion_ctrl_mode_ = mode;
    motion_ctrl_owner_ = caller_id;
    RCLCPP_DEBUG(node_->get_logger(), "Motion control mode transition (%s -> %s)", MOTION_CTRL_MODE_NAMES.at(last_mode),
                    MOTION_CTRL_MODE_NAMES.at(mode));
    return true;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Motion control mode transition (%s -> %s) not possible!",
                            MOTION_CTRL_MODE_NAMES.at(last_mode), MOTION_CTRL_MODE_NAMES.at(mode));
    return false;
  }
}

void RobotDriver::setJointPositions(const std::array<double, MAX_NUM_JOINTS>& position, rclcpp::Time& timestamp)
{
  assert(num_joints_ <= position.size() && "Count of active joints exceeds expected maximum count of joints");

  std::lock_guard<std::mutex> lock(state_mtx_);
  state_timestamp_ = timestamp;
  robot_state_.setJointGroupPositions(joint_model_group_, position.data());
  robot_state_.updateLinkTransforms();
  state_update_cv_.notify_all();
}

void RobotDriver::setJointPositionsAndVelocities(const std::array<double, MAX_NUM_JOINTS>& position,
                                                 const std::array<double, MAX_NUM_JOINTS>& velocity,
                                                 rclcpp::Time& timestamp)
{
  assert(num_joints_ <= position.size() && "Count of active joints exceeds expected maximum count of joints");
  assert(num_joints_ <= velocity.size() && "Count of active joints exceeds expected maximum count of joints");

  std::lock_guard<std::mutex> lock(state_mtx_);
  state_timestamp_ = timestamp;
  robot_state_.setJointGroupPositions(joint_model_group_, position.data());
  robot_state_.setJointGroupVelocities(joint_model_group_, velocity.data());
  robot_state_.updateLinkTransforms();
  state_update_cv_.notify_all();
}

void RobotDriver::setRobotStatus(RobotStatus& status, rclcpp::Time& timestamp)
{
  std::lock_guard<std::mutex> lock(status_mtx_);
  status_timestamp_ = timestamp;
  robot_status_ = status;
}

bool RobotDriver::waitForStateUpdate(rclcpp::Time& last_update_time, int timeout_ms)
{
  std::unique_lock<std::mutex> lock(state_mtx_);
  bool rtn = waitForStateUpdate_internal(lock, last_update_time, timeout_ms);

  return rtn;
}

bool RobotDriver::waitForFeedback(const std::string& tcp_frame, const std::string& reference_frame,
                                  rclcpp::Time& last_feedback_time, int timeout_ms, Eigen::Isometry3d& trsf)
{
  std::unique_lock<std::mutex> lock(state_mtx_);

  if (!waitForStateUpdate_internal(lock, last_feedback_time, timeout_ms))
  {
    RCLCPP_WARN(node_->get_logger(), "No feedback received for %d ms", timeout_ms);
    return false;
  }

  return getTransform_internal(tcp_frame, reference_frame, trsf);
}

bool RobotDriver::startJogging(uint8_t type)
{
  return startVelocityControl(type, MotionControlMode::JOGGING);
}

bool RobotDriver::startPoseTracking(uint8_t type)
{
  return startVelocityControl(type, MotionControlMode::POSE_TRACKING);
}

bool RobotDriver::stopMotionControl()
{
  // set internal motion control mode to IDLE
  return setMotionControlMode(MotionControlMode::IDLE);
}

bool RobotDriver::sendVelocityCommand(const motion_control_msgs::msg::VelocityCommand& vel_cmd)
{
  motion_control_msgs::msg::VelocityCommand vel_cmd_filt = vel_cmd;

  if (!enforceLimits(vel_cmd_filt))
    return false;

  // send velocity command
  bool rtn = sendVelocityCommand_internal(vel_cmd_filt);
  if (!rtn) RCLCPP_ERROR(node_->get_logger(), "Failed to send velocity command");
  RCLCPP_DEBUG(node_->get_logger(), "motion_cmd", "Sent velocity command");

  // publish raw/filtered velocity command (if enabled)
  if (publish_vel_cmd_)
  {
    rclcpp::Time timestamp = node_->now();
    motion_control_msgs::msg::VelocityCommand vel_cmd_raw = vel_cmd;
    vel_cmd_raw.header.stamp = timestamp;
    vel_cmd_filt.header.stamp = timestamp;
    raw_vel_cmd_publisher_->publish(vel_cmd_raw);
    filt_vel_cmd_publisher_->publish(vel_cmd_filt);
  }

  return rtn;
}

void RobotDriver::computeCartesianLimitsScale(const Vector& cart_vel, double& linear_scale, double& angular_scale)
{
  linear_scale = 1.0;
  angular_scale = 1.0;

  double v_linear = cart_vel.segment<3>(0).norm();
  double v_angular = cart_vel.segment<3>(3).norm();

  if (v_linear > vel_ctrl_settings_->cartesian_limits.max_linear_velocity)
  {
    linear_scale = vel_ctrl_settings_->cartesian_limits.max_linear_velocity / v_linear;
    RCLCPP_WARN(node_->get_logger(), "Limiting cartesian linear velocity (%.2f m/s)",
                            vel_ctrl_settings_->cartesian_limits.max_linear_velocity);
  }

  if (v_angular > vel_ctrl_settings_->cartesian_limits.max_angular_velocity)
  {
    angular_scale = vel_ctrl_settings_->cartesian_limits.max_angular_velocity / v_angular;
    RCLCPP_WARN(node_->get_logger(), "Limiting cartesian angular velocity (%.1f deg/s)",
                            angles::to_degrees(vel_ctrl_settings_->cartesian_limits.max_angular_velocity));
  }
}

void RobotDriver::computeJointLimitsScale(const Vector& joint_pos, const Vector& joint_vel, double& scale)
{
  scale = 1.0;
  int excessive_joint_index = -1;
  double excessive_joint_max_velocity = 0.0;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // check position
    if (joint_limits_[i].position_bounded_)
    {
      double lower_limit = joint_limits_[i].min_position_ + vel_ctrl_settings_->joint_limits.stop_tolerance;
      double upper_limit = joint_limits_[i].max_position_ - vel_ctrl_settings_->joint_limits.stop_tolerance;

      if ((joint_pos[i] <= lower_limit && joint_vel[i] < 0) || (joint_pos[i] >= upper_limit && joint_vel[i] > 0))
      {
        // stop to avoid hitting position limit
        scale = 0.0;
        // clang-format off
        RCLCPP_WARN(node_->get_logger(), 
                                "Position limit reached for joint '%s' "
                                "(pos: %.1fdeg, lower: %.1fdeg, upper: %.1fdeg, tol: %.1fdeg, vel: %.1fdeg/s)",
                                joint_names_[i].c_str(), angles::to_degrees(joint_pos[i]),
                                angles::to_degrees(joint_limits_[i].min_position_),
                                angles::to_degrees(joint_limits_[i].max_position_),
                                angles::to_degrees(vel_ctrl_settings_->joint_limits.stop_tolerance),
                                angles::to_degrees(joint_vel[i]));
        // clang-format on

        return;
      }
    }

    // check velocity (ignoring min_velocity, assuming min_velocity = -max_velocity)
    if (joint_limits_[i].velocity_bounded_ || vel_ctrl_settings_->joint_limits.has_velocity_limit)
    {
      // determine the actual maximum velocity to use
      double max_velocity = vel_ctrl_settings_->joint_limits.has_velocity_limit ?
                                std::min(vel_ctrl_settings_->joint_limits.max_velocity, joint_limits_[i].max_velocity_) :
                                joint_limits_[i].max_velocity_;

      if (std::abs(joint_vel[i]) > max_velocity)
      {
        double joint_scale = max_velocity / std::abs(joint_vel[i]);
        if (joint_scale < scale)
        {
          // update scale for most excessive joint velocity
          scale = joint_scale;
          excessive_joint_index = i;
          excessive_joint_max_velocity = max_velocity;
        }
      }
    }
  }

  if (excessive_joint_index != -1)
  {
    // Note: Only most excessive joint is mentioned...
    RCLCPP_WARN(node_->get_logger(), "Limiting velocity in joint '%s' (%.1f deg/s)",
                            joint_names_[excessive_joint_index].c_str(),
                            angles::to_degrees(excessive_joint_max_velocity));
  }
}

bool RobotDriver::enforceLimits(motion_control_msgs::msg::VelocityCommand& vel_cmd)
{
  // get feedback joint position and jacobian matrix
  Vector joint_pos(num_joints_);
  Matrix jacobian(6, num_joints_);
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    robot_state_.copyJointGroupPositions(joint_model_group_, joint_pos.data());
    // FIXME: compute jacobian without dynamic memory allocation (?)
    jacobian = robot_state_.getJacobian(joint_model_group_);
  }

  if (vel_cmd.type == motion_control_msgs::msg::VelocityCommand::BASE_FRAME ||
      vel_cmd.type == motion_control_msgs::msg::VelocityCommand::TOOL_FRAME)
  {
    // enforce cartesian velocity limits
    auto cart_vel_cmd = Eigen::Map<Vector>(vel_cmd.cmd.data(), 6);
    double linear_scale, angular_scale;
    computeCartesianLimitsScale(cart_vel_cmd, linear_scale, angular_scale);
    cart_vel_cmd.segment<3>(0) *= linear_scale;
    cart_vel_cmd.segment<3>(3) *= angular_scale;
  }
  else if (vel_cmd.type == motion_control_msgs::msg::VelocityCommand::JOINT)
  {
    // enforce joint position/velocity limits
    auto joint_vel_cmd = Eigen::Map<Vector>(vel_cmd.cmd.data(), num_joints_);
    double scale;
    computeJointLimitsScale(joint_pos, joint_vel_cmd, scale);
    joint_vel_cmd *= scale;

    if (scale > 0)
    {
      // enforce cartesian velocity limits
      Vector cart_vel(6);
      cart_vel = jacobian * joint_vel_cmd;
      double linear_scale, angular_scale;
      computeCartesianLimitsScale(cart_vel, linear_scale, angular_scale);
      joint_vel_cmd *= std::min(linear_scale, angular_scale);
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Velocity command with unknown type (%u)", vel_cmd.type);
    return false;
  }

  return true;
}

bool RobotDriver::getTransform_internal(const std::string& tcp_frame, const std::string& reference_frame,
                                        Eigen::Isometry3d& trsf)
{
  const std::string& actual_tcp_frame = tcp_frame.empty() ? vel_ctrl_settings_->tool_frame : tcp_frame;
  const std::string& actual_reference_frame = reference_frame.empty() ? vel_ctrl_settings_->base_frame : reference_frame;

  if (tcp_frame.empty())
    RCLCPP_WARN_ONCE(node_->get_logger(), "No TCP frame specified. Using the tool frame '%s' as TCP frame by default",
                        actual_tcp_frame.c_str());

  if (reference_frame.empty())
    RCLCPP_WARN_ONCE(node_->get_logger(), 
                        "No reference frame specified. Using the base frame '%s' as reference frame by default",
                        actual_reference_frame.c_str());

  if (!robot_state_.knowsFrameTransform(actual_tcp_frame))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current transform. The TCP frame '%s' is not known",
                    actual_tcp_frame.c_str());
    return false;
  }

  if (!robot_state_.knowsFrameTransform(actual_reference_frame))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current transform. The reference frame '%s' is not known",
                    actual_reference_frame.c_str());
    return false;
  }

  trsf = robot_state_.getGlobalLinkTransform(actual_reference_frame).inverse() *
         robot_state_.getGlobalLinkTransform(actual_tcp_frame);

  return true;
}

bool RobotDriver::sendRequest(industrial::typed_message::TypedMessage& typed_msg)
{
  SimpleMessage msg;
  if (!typed_msg.toRequest(msg))
    return false;

  return sendRequest(msg);
}

bool RobotDriver::sendRequest(SimpleMessage& msg)
{
  SimpleMessage reply;
  if (!motion_client_manager_->sendAndReceiveMsg(msg, reply))
    return false;

  if(!reply.validateMessage()) RCLCPP_DEBUG(node_->get_logger(), "Simple message validation failed!");
  if(reply.getReplyCode() == ReplyType::FAILURE) RCLCPP_DEBUG(node_->get_logger(), "Received reply code 'FAILURE'");

  return reply.getReplyCode() == ReplyType::SUCCESS;
}

bool RobotDriver::startVelocityControl(uint8_t type, MotionControlMode mode)
{
  // check connection state
  if (!isConnected())
  {
    RCLCPP_ERROR(node_->get_logger(), "Starting velocity control mode not possible. Not connected to the robot.");
    return false;
  }

  // check if motion possible
  if (!isMotionPossible())
  {
    RCLCPP_ERROR(node_->get_logger(), "Refusing to start velocity control mode. Motion not possible on robot controller. "
                             "Check the robot status message.");
    return false;
  }

  // try to lock the motion control mode
  if (!setMotionControlMode(MotionControlMode::LOCKED))
    return false;

  // try to start velocity control mode:
  // velocity config message triggers motion control mode transition on the robot controller side
  if (!sendVelocityConfig_internal(type))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to enable velocity control mode on the robot controller");
    setMotionControlMode(MotionControlMode::IDLE);
    return false;
  }

  setMotionControlMode(mode);

  return true;
}

bool RobotDriver::waitForStateUpdate_internal(std::unique_lock<std::mutex>& lock, rclcpp::Time& last_update_time,
                                              int timeout_ms)
{
  bool timeout = !state_update_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                            [&]() { return state_timestamp_ > last_update_time; });

  if (timeout)
    return false;

  last_update_time = state_timestamp_;

  return true;
}

}  // namespace robot_middleware
