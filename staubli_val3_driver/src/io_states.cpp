
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

#include "staubli_val3_driver/io_states.hpp"

#include "simple_message/shared_types.hpp"
#include "rclcpp/rclcpp.hpp"

IOStates::IOStates()
{
}

IOStates::~IOStates()
{
}

bool IOStates::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("io_states"), "Executing IO states load");

  if (buffer->load(this->user_in_) && buffer->load(this->valve_out_) && buffer->load(this->basic_in_) &&
      buffer->load(this->basic_out_) && buffer->load(this->basic_in_2_) && buffer->load(this->basic_out_2_) &&
      buffer->load(this->basic_io_valid_) && buffer->load(this->basic_io_2_valid_))
  {

    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states not loaded");
    rtn = false;
  }

  return rtn;
}

bool IOStates::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("io_states"), "Executing IO states unload");
  if (buffer->unload(this->basic_io_2_valid_) && buffer->unload(this->basic_io_valid_) &&
      buffer->unload(this->basic_out_2_) && buffer->unload(this->basic_in_2_) && buffer->unload(this->basic_out_) &&
      buffer->unload(this->basic_in_) && buffer->unload(this->valve_out_) && buffer->unload(this->user_in_))
  {

    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("io_states"), "Failed to unload IO states");
    rtn = false;
  }

  return rtn;
}
