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

#include "simple_message/messages/velocity_config_message.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace velocity_config_message
{

VelocityConfigMessage::VelocityConfigMessage()
{
  this->init();
}

VelocityConfigMessage::~VelocityConfigMessage()
{
}

void VelocityConfigMessage::init()
{
  this->setMessageType(1640);
  this->data_.init();
}

void VelocityConfigMessage::init(const industrial::velocity_config::VelocityConfig& data)
{
  this->data_ = data;
}

bool VelocityConfigMessage::init(SimpleMessage& msg)
{
  this->init();
  ByteArray data = msg.getData();
  if (!data.unload(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config_message"), "Failed to unload velocity config message data");
    return false;
  }

  return true;
}

bool VelocityConfigMessage::load(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config_message"), "Executing velocity config message load");

  if (!buffer->load(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config_message"), "Failed to load velocity config message data");
    return false;
  }

  return true;
}

bool VelocityConfigMessage::unload(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config_message"), "Executing velocity config message unload");

  if (!buffer->unload(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config_message"), "Failed to unload velocity config message data");
    return false;
  }

  return true;
}

}  // namespace industrial
}  // namespace velocity_config_message
