
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

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control and https://github.com/ros-industrial/staubli_val3_driver/tree/master
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include "staubli_val3_driver/set_drive_power_message.hpp"

#include "simple_message/byte_array.hpp"
#include "rclcpp/rclcpp.hpp"

SetDrivePowerMessage::SetDrivePowerMessage()
{
  this->init();
}

SetDrivePowerMessage::~SetDrivePowerMessage()
{
}

bool SetDrivePowerMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->drive_power_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("set_drive_power_message"), "Failed to unload SetDrivePower data");
  }
  return rtn;
}

void SetDrivePowerMessage::init()
{
  this->setMessageType(1610);  // TODO: make enum for Stäubli specific standard port numbers
}

void SetDrivePowerMessage::init(industrial::shared_types::shared_bool drive_power)
{
  this->init();
  this->drive_power_ = drive_power;
}

bool SetDrivePowerMessage::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "Executing SetDrivePower load");

  if (buffer->load(this->drive_power_))
  {

    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower not loaded");
    rtn = false;
  }

  return rtn;
}

bool SetDrivePowerMessage::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "Executing SetDrivePower unload");
  if (buffer->unload(this->drive_power_))
  {

    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("set_drive_power_message"), "Failed to unload SetDrivePower");
    rtn = false;
  }

  return rtn;
}