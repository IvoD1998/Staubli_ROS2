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

#include "simple_message/velocity_config.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace velocity_config
{

VelocityConfig::VelocityConfig()
{
  this->init();
}

VelocityConfig::~VelocityConfig()
{
}

void VelocityConfig::init()
{
  this->cmd_type_ = 0;
  std::memset(this->frame_ref_, 0, sizeof(this->frame_ref_));
  std::memset(this->tool_ref_, 0, sizeof(this->tool_ref_));
  this->accel_ = 0.0;
  this->vel_ = 0.0;
  this->tvel_ = 0.0;
  this->rvel_ = 0.0;
}

bool VelocityConfig::load(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config"), "Executing velocity config load");

  if (!buffer->load(this->cmd_type_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config cmd_type");
    return false;
  }

  for (const shared_real& value : this->frame_ref_)
  {
    if (!buffer->load(value))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config frame_ref");
      return false;
    }
  }

  for (const shared_real& value : this->tool_ref_)
  {
    if (!buffer->load(value))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config tool_ref");
      return false;
    }
  }

  if (!buffer->load(accel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config accel");
    return false;
  }

  if (!buffer->load(vel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config vel");
    return false;
  }

  if (!buffer->load(this->tvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config tvel");
    return false;
  }

  if (!buffer->load(this->rvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config rvel");
    return false;
  }

  return true;
}

bool VelocityConfig::unload(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config"), "Executing velocity config unload");

  if (!buffer->unload(this->rvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config rvel");
    return false;
  }

  if (!buffer->unload(this->tvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config tvel");
    return false;
  }

  if (!buffer->unload(this->vel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config vel");
    return false;
  }

  if (!buffer->unload(this->accel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config accel");
    return false;
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->tool_ref_[i]))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config tool_ref");
      return false;
    }
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->frame_ref_[i]))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config frame_ref");
      return false;
    }
  }

  if (!buffer->unload(this->cmd_type_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config cmd_type");
    return false;
  }

  return true;
}

}  // namespace industrial
}  // namespace velocity_config