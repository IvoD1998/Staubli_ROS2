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
#include "simple_message/message_handler.hpp"
#else
#include "message_handler.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace message_handler
{

using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;

MessageHandler::MessageHandler(void)
{
  this->setConnection(NULL);
  this->setMsgType(StandardMsgTypes::INVALID);
}


MessageHandler::~MessageHandler(void)
{
}


bool MessageHandler::init(int msg_type, SmplMsgConnection* connection)
{
  bool rtn = false;
  
  if (StandardMsgTypes::INVALID != msg_type)
  {
    if (NULL != connection)
    {
      this->setConnection(connection);
      this->setMsgType(msg_type);
      rtn = true;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Message connection is NULL");
      rtn = false;
    }
    }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Message handler type: %d, not valid", msg_type);
    rtn = false;
    }
    
  return rtn;
}

    
    
bool MessageHandler::callback(SimpleMessage & in)
{
  bool rtn = false;
  
  if (validateMsg(in))
  {
    this->internalCB(in);
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Invalid message passed to callback");
    rtn = true;
  }
  
  return rtn;
}


bool MessageHandler::validateMsg(SimpleMessage & in)
{
  bool rtn = false;
  
  if (in.validateMessage())
  {
    if (in.getMessageType() == this->getMsgType())
    {
      rtn = true;
    }
    else
    {
      //RCLCPP_WARN(rclcpp::get_logger("message_handler"), "Message type: %d, doesn't match handler type: %d", in.getMessageType(), this->getMsgType());
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("message_handler"), "Passed in message invalid");
  }

  return rtn;
  
}

} // namespace message_handler
} // namespace industrial
