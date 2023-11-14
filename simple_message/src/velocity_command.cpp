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

#include "simple_message/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace velocity_command
{

    VelocityCommand::VelocityCommand()
    {
        this->init();
    }

    VelocityCommand::~VelocityCommand()
    {
    }

    void VelocityCommand::init()
    {
        this->sequence_ = 0;
        this->type_ = 0;
        std::memset(this->vector_, 0, sizeof(this->vector_));
    }

    bool VelocityCommand::load(ByteArray *buffer)
    {
        //RCLCPP_INFO(rclcpp::get_logger("velocity_command"), "Executing velocity command load");

        if (!buffer->load(this->sequence_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command sequence");
            return false;
        }

        for (const shared_real &value : this->vector_)
        {
            if (!buffer->load(value))
            {
                //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command vector");
                return false;
            }
        }

        if (!buffer->load(this->type_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command type");
            return false;
        }

        return true;
    }

    bool VelocityCommand::unload(ByteArray *buffer)
    {
        //RCLCPP_INFO(rclcpp::get_logger("velocity_command"), "Executing velocity command unload");

        if (!buffer->unload(this->type_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command type");
            return false;
        }

        for (int i = MAX_NUM_JOINTS - 1; i >= 0; i--)
        {
            if (!buffer->unload(this->vector_[i]))
            {
                //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command vector");
                return false;
            }
        }

        if (!buffer->unload(this->sequence_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command sequence");
            return false;
        }

        return true;
    }

} // namespace industrial
} // namespace velocity_command
