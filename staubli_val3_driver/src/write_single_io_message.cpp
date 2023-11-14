
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

#include "staubli_val3_driver/write_single_io_message.hpp"

#include "simple_message/byte_array.hpp"
#include "rclcpp/rclcpp.hpp"

WriteSingleIOMessage::WriteSingleIOMessage()
{
  this->init();
}

WriteSingleIOMessage::~WriteSingleIOMessage()
{
}

bool WriteSingleIOMessage::init(industrial::simple_message::SimpleMessage &msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io_message"), "Failed to unload write single IO data");
  }
  return rtn;
}

void WriteSingleIOMessage::init(WriteSingleIO& writeSingleIO)
{
  this->init();
  this->writeSingleIO_.copyFrom(writeSingleIO);
}

void WriteSingleIOMessage::init()
{
  this->setMessageType(1621);  // TODO: make enum for Stäubli specific standard port numbers
  this->writeSingleIO_.init();
}

bool WriteSingleIOMessage::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn;
  RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Executing write single IO message load");

  if (buffer->load(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io_message"), "Failed to load write single IO data");
  }
  return rtn;
}

bool WriteSingleIOMessage::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn;
  RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Executing write single IO message unload");

  if (buffer->unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Failed to unload write single IO data");
  }
  return rtn;
}