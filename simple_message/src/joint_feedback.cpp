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
#include "simple_message/joint_feedback.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_feedback.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::joint_data;
using namespace industrial::shared_types;

namespace industrial
{
namespace joint_feedback
{

JointFeedback::JointFeedback(void)
{
  this->init();
}
JointFeedback::~JointFeedback(void)
{

}

void JointFeedback::init()
{
  this->robot_id_ = 0;
  this->valid_fields_ = 0;
  this->time_ = 0.0;
  this->positions_.init();
  this->velocities_.init();
  this->accelerations_.init();
}

void JointFeedback::init(industrial::shared_types::shared_int robot_id,
          industrial::shared_types::shared_int valid_fields,
          industrial::shared_types::shared_real time,
          industrial::joint_data::JointData & positions,
          industrial::joint_data::JointData & velocities,
          industrial::joint_data::JointData & accelerations)
{
  this->setRobotID(robot_id);
  this->setTime(time);
  this->setPositions(positions);
  this->setVelocities(velocities);
  this->setAccelerations(accelerations);
  this->valid_fields_ = valid_fields;  // must happen after others are set
}

void JointFeedback::copyFrom(JointFeedback &src)
{
  this->setRobotID(src.getRobotID());
  src.getTime(this->time_);
  src.getPositions(this->positions_);
  src.getVelocities(this->velocities_);
  src.getAccelerations(this->accelerations_);
  this->valid_fields_ = src.valid_fields_;
}

bool JointFeedback::operator==(JointFeedback &rhs)
{
  return this->robot_id_ == rhs.robot_id_ &&
         this->valid_fields_ == rhs.valid_fields_ &&
         ( !is_valid(ValidFieldTypes::TIME) || (this->time_ == rhs.time_) ) &&
         ( !is_valid(ValidFieldTypes::POSITION) || (this->positions_ == rhs.positions_) ) &&
         ( !is_valid(ValidFieldTypes::VELOCITY) || (this->velocities_ == rhs.velocities_) ) &&
         ( !is_valid(ValidFieldTypes::ACCELERATION) || (this->accelerations_ == rhs.accelerations_) );
}

bool JointFeedback::load(industrial::byte_array::ByteArray *buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("joint_feedback"), "Executing joint feedback load");

  if (!buffer->load(this->robot_id_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback robot_id");
    return false;
  }

  if (!buffer->load(this->valid_fields_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback valid fields");
    return false;
  }

  if (!buffer->load(this->time_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback time");
    return false;
  }

  if (!this->positions_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback positions");
    return false;
  }

  if (!this->velocities_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback velocities");
    return false;
  }

  if (!this->accelerations_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to load joint feedback accelerations");
    return false;
  }

  //RCLCPP_INFO(rclcpp::get_logger("joint_feedback"), "Joint feedback successfully loaded");
  return true;
}

bool JointFeedback::unload(industrial::byte_array::ByteArray *buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("joint_feedback"), "Executing joint feedback unload");

  if (!this->accelerations_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to unload joint feedback accelerations");
    return false;
  }

  if (!this->velocities_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to unload joint feedback velocities");
    return false;
  }

  if (!this->positions_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to unload joint feedback positions");
    return false;
  }

  if (!buffer->unload(this->time_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to unload joint feedback time");
    return false;
  }

  if (!buffer->unload(this->valid_fields_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Failed to unload joint feedback valid fields");
    return false;
  }

  if (!buffer->unload(this->robot_id_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback"), "Faild to unload joint feedback robot_id");
    return false;
  }

  //RCLCPP_INFO(rclcpp::get_logger("joint_feedback"), "Joint feedback successfully unloaded");
  return true;
}

}
}

