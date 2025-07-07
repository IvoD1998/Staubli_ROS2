/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Modified from original version in https://github.com/ros-industrial/industrial_core
// Changes made to support ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#ifndef FLATHEADERS
#include "simple_message/robot_status.hpp"
#include "simple_message/shared_types.hpp"

#else
#include "robot_status.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
// Files below used to translate between ROS messages enums and
// enums defined in this file
#include "industrial_msgs/msg/robot_mode.hpp"
#include "industrial_msgs/msg/tri_state.hpp"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace robot_status
{

namespace RobotModes
{

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)

int toROSMsgEnum(RobotModes::RobotMode mode)
{

  switch (mode)
  {
    case RobotModes::AUTO:
      return industrial_msgs::msg::RobotMode::AUTO;
      break;
    case RobotModes::MANUAL:
      return industrial_msgs::msg::RobotMode::MANUAL;
      break;
    case RobotModes::UNKNOWN:
      return industrial_msgs::msg::RobotMode::UNKNOWN;
  }
  return industrial_msgs::msg::RobotMode::UNKNOWN;

}
;

#endif

}

namespace TriStates
{

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)

int toROSMsgEnum(TriStates::TriState state)
{

  switch (state)
  {
    case TriStates::TS_UNKNOWN:
      return industrial_msgs::msg::TriState::UNKNOWN;
      break;
    case TriStates::TS_TRUE:
      return industrial_msgs::msg::TriState::TRUE;
      break;
    case TriStates::TS_FALSE:
      return industrial_msgs::msg::TriState::FALSE;
      break;
  }
  return industrial_msgs::msg::TriState::UNKNOWN;

}
;

#endif

}

RobotStatus::RobotStatus(void)
{
  this->init();
}
RobotStatus::~RobotStatus(void)
{

}

void RobotStatus::init()
{
  this->init(TriStates::TS_UNKNOWN, TriStates::TS_UNKNOWN, 0, TriStates::TS_UNKNOWN,
             TriStates::TS_UNKNOWN, RobotModes::UNKNOWN, TriStates::TS_UNKNOWN, TriStates::TS_UNKNOWN);
}

void RobotStatus::init(TriState drivesPowered, TriState eStopped, industrial::shared_types::shared_int errorCode,
                       TriState inError, TriState inMotion, RobotMode mode, TriState motionPossible, TriState trajectoryComplete)
{
  this->setDrivesPowered(drivesPowered);
  this->setEStopped(eStopped);
  this->setErrorCode(errorCode);
  this->setInError(inError);
  this->setInMotion(inMotion);
  this->setMode(mode);
  this->setMotionPossible(motionPossible);
  this->setTrajectoryComplete(trajectoryComplete);
}

void RobotStatus::copyFrom(RobotStatus &src)
{
  this->setDrivesPowered(src.getDrivesPowered());
  this->setEStopped(src.getEStopped());
  this->setErrorCode(src.getErrorCode());
  this->setInError(src.getInError());
  this->setInMotion(src.getInMotion());
  this->setMode(src.getMode());
  this->setMotionPossible(src.getMotionPossible());
  this->setTrajectoryComplete(src.getTrajectoryComplete());
}

bool RobotStatus::operator==(RobotStatus &rhs)
{
  return this->drives_powered_ == rhs.drives_powered_ && this->e_stopped_ == rhs.e_stopped_
      && this->error_code_ == rhs.error_code_ && this->in_error_ == rhs.in_error_ && this->in_motion_ == rhs.in_motion_
      && this->mode_ == rhs.mode_ && this->motion_possible_ == rhs.motion_possible_ && this->trajectory_complete_ == rhs.trajectory_complete_;
}

bool RobotStatus::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Executing robot status load");

  if (buffer->load(this->drives_powered_) && buffer->load(this->e_stopped_) && buffer->load(this->error_code_)
      && buffer->load(this->in_error_) && buffer->load(this->in_motion_) && buffer->load(this->mode_)
      && buffer->load(this->motion_possible_) && buffer->load(this->trajectory_complete_))
  {

    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status successfully loaded");
    rtn = true;
  }
  else
  {
    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status not loaded");
    rtn = false;
  }

  return rtn;
}

bool RobotStatus::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Executing robot status unload");
  if (buffer->unload(this->trajectory_complete_) && buffer->unload(this->motion_possible_) && buffer->unload(this->mode_) && buffer->unload(this->in_motion_)
      && buffer->unload(this->in_error_) && buffer->unload(this->error_code_) && buffer->unload(this->e_stopped_)
      && buffer->unload(this->drives_powered_))
  {

    rtn = true;
    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status successfully unloaded");
  }

  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("robot_status"), "Failed to unload robot status");
    rtn = false;
  }

  return rtn;
}

}
}

