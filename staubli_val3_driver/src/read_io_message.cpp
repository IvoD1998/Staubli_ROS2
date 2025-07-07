
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

#include "staubli_val3_driver/read_io_message.hpp"

#include "simple_message/byte_array.hpp"
#include "simple_message/simple_message.hpp"

#include "rclcpp/rclcpp.hpp"

ReadIOMessage::ReadIOMessage()
{
  this->init();
}

ReadIOMessage::~ReadIOMessage()
{
}

bool ReadIOMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to unload read IO data");
  }
  return rtn;
}

void ReadIOMessage::init()
{
  this->setMessageType(1620);  // TODO: make enum for Stäubli specific standard port numbers
}

bool ReadIOMessage::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;
  RCLCPP_INFO(rclcpp::get_logger("read_io_message"), "Executing read IO message load");
  if (buffer->load(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to read IO states data");
  }
  return rtn;
}

bool ReadIOMessage::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;
  RCLCPP_INFO(rclcpp::get_logger("read_io_message"), "Executing read IO message unload");

  if (buffer->unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to unload read IO data");
  }
  return rtn;
}