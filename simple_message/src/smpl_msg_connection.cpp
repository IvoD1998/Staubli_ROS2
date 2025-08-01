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
#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/byte_array.hpp"
#else
#include "smpl_msg_connection.hpp"
#include "byte_array.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

#ifdef SIMPLE_MESSAGE_MOTOPLUS
#include "motoPlus.h"
#endif

using namespace industrial::simple_message;
using namespace industrial::shared_types;
using namespace industrial::byte_array;

namespace industrial
{

namespace smpl_msg_connection
{


bool SmplMsgConnection::sendMsg(SimpleMessage & message)
{
  bool rtn;
  ByteArray sendBuffer;
  ByteArray msgData;

  if (message.validateMessage())
  {
    message.toByteArray(msgData);
    sendBuffer.load((int)msgData.getBufferSize());
    sendBuffer.load(msgData);
    rtn = this->sendBytes(sendBuffer);
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Message validation failed, message not sent");
  }

return rtn;
}


bool SmplMsgConnection::receiveMsg(SimpleMessage & message)
{
  return receiveMsg(message, -1);
}


bool SmplMsgConnection::receiveMsg(SimpleMessage & message, shared_int timeout_ms)
{
  ByteArray lengthBuffer;
  ByteArray msgBuffer;
  int length;

  bool rtn = false;


  rtn = this->receiveBytes(lengthBuffer, message.getLengthSize(), timeout_ms);

  if (rtn)
  {
    rtn = lengthBuffer.unload(length);
    //RCLCPP_INFO(rclcpp::get_logger("smpl_msg_connection"), "Message length: %d", length);

    if (rtn)
    {
      rtn = this->receiveBytes(msgBuffer, length, timeout_ms);

      if (rtn)
      {
        rtn = message.init(msgBuffer);
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to initialize message");
        rtn = false;
      }

    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to receive message");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to receive message length");
    rtn = false;
  }

  return rtn;
}


bool SmplMsgConnection::sendAndReceiveMsg(SimpleMessage & send, SimpleMessage & recv, bool verbose)
{
  return sendAndReceiveMsg(send, recv, -1, verbose);
}

bool SmplMsgConnection::sendAndReceiveMsg(SimpleMessage & send, SimpleMessage & recv,
                                          shared_int timeout_ms, bool verbose)
{
  bool rtn = false;
  rtn = this->sendMsg(send);
  if (rtn)
  {
    if(verbose) {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Sent message");
    }
    rtn = this->receiveMsg(recv, timeout_ms);
    if(verbose) {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Got message");
    }
  }
  else
  {
    rtn = false;
  }

  return rtn;
}


}//smpl_msg_connection
}//industrial
