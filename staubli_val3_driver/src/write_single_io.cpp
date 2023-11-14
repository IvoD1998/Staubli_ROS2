
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

#include "staubli_val3_driver/write_single_io.hpp"
#include "rclcpp/rclcpp.hpp"

WriteSingleIO::WriteSingleIO()
{
  this->init();
}

WriteSingleIO::~WriteSingleIO()
{
}

void WriteSingleIO::init()
{
  this->init(IOModule::UNKNOWN, 0, false);
}

void WriteSingleIO::init(IOModule module_id, 
                         industrial::shared_types::shared_int pin, 
                         industrial::shared_types::shared_bool state)
{
  this->setModuleId(module_id);
  this->setPin(pin);
  this->setState(state);
}

void WriteSingleIO::copyFrom(WriteSingleIO& src)
{
  this->init(src.getModuleId(), src.getPin(), src.getState());
}

bool WriteSingleIO::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("write_single_io"),"Executing WriteSingleIO load");

  if (buffer->load(this->module_id_) && buffer->load(this->pin_) && buffer->load(this->state_))
  {

    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO not loaded");
    rtn = false;
  }

  return rtn;
}

bool WriteSingleIO::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "Executing WriteSingleIO unload");
  if (buffer->unload(this->state_) && buffer->unload(this->pin_) && buffer->unload(this->module_id_))
  {
    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io"), "Failed to unload WriteSingleIO");
    rtn = false;
  }

  return rtn;
}