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
#include "simple_message/message_manager.hpp"
#include "simple_message/simple_message.hpp"
#else
#include "message_manager.hpp"
#include "simple_message.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
#else
#include "unistd.h"
#endif

#include <thread>
#include <chrono>

using namespace industrial::smpl_msg_connection;
using namespace industrial::message_handler;
using namespace industrial::simple_message;
using namespace industrial::comms_fault_handler;
using namespace industrial::simple_comms_fault_handler;

namespace industrial
{
namespace message_manager
{

/**
 * \brief Constructor
 */
MessageManager::MessageManager()
{
  this->num_handlers_ = 0;
  for (unsigned int i = 0; i < this->getMaxNumHandlers(); i++)
  {
    this->handlers_[i] = NULL;
  }
  this->comms_hndlr_ = NULL;
}

MessageManager::~MessageManager()
{

}

bool MessageManager::init(SmplMsgConnection* connection)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Initializing message manager with default comms fault handler");


  if (NULL != connection)
  {
    this->getDefaultCommsFaultHandler().init(connection);
    this->init(connection, (CommsFaultHandler*)(&this->getDefaultCommsFaultHandler()));
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL connection passed into manager init");
    rtn = false;
  }

  return rtn;
}

bool MessageManager::init(SmplMsgConnection* connection, CommsFaultHandler* fault_handler)
{
  bool rtn = false;

    //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Initializing message manager");

    if (NULL != connection && NULL != fault_handler)
    {
      this->setConnection(connection);
      this->getPingHandler().init(connection);
      this->setCommsFaultHandler(fault_handler);

      if (this->add(&this->getPingHandler()))
      {
        rtn = true;
      }
      else
      {
        rtn = false;
        //RCLCPP_WARN(rclcpp::get_logger("message_manager"), "Failed to add ping handler, manager won't respond to pings");
      }

    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL connection or NULL fault handler passed into manager init");
      rtn = false;
    }

    return rtn;
  }



void MessageManager::spinOnce()
{
  SimpleMessage msg;
  MessageHandler* handler = NULL;

  if(!this->getConnection()->isConnected())
  {
    this->getCommsFaultHandler()->connectionFailCB();
  }

  if (this->getConnection()->receiveMsg(msg))
  {
    //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Message received");
    handler = this->getHandler(msg.getMessageType());

    if (NULL != handler)
    {
      //RCLCPP_DEBUG(rclcpp::get_logger("message_manager"), "Executing handler callback for message type: %d", handler->getMsgType());
      handler->callback(msg);
    }
    else
    {
      if (CommTypes::SERVICE_REQUEST == msg.getCommType())
      {
        simple_message::SimpleMessage fail;
        fail.init(msg.getMessageType(), CommTypes::SERVICE_REPLY, ReplyTypes::FAILURE);
        this->getConnection()->sendMsg(fail);
        //RCLCPP_WARN(rclcpp::get_logger("message_manager"), "Unhandled message type encounters, sending failure reply");
      }
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Message callback for message type: %d, not executed", msg.getMessageType());
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Failed to receive incoming message");
    this->getCommsFaultHandler()->sendFailCB();
  }
}

int ms_per_clock;
void mySleep(int sec)
{
#ifdef SIMPLE_MESSAGE_MOTOPLUS
  if (ms_per_clock <= 0)
    ms_per_clock = mpGetRtc();

  mpTaskDelay(sec * 1000 / ms_per_clock);
#else
  std::this_thread::sleep_for(std::chrono::seconds(sec));
#endif
}

void MessageManager::spin()
{
  //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Entering message manager spin loop");
// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
  while (rclcpp::ok())
#else
  while (true)
#endif
  {
    this->spinOnce();

    // Throttle loop speed if waiting for a re-connection
    if (!this->getConnection()->isConnected())
      mySleep(5);
  }
}

bool MessageManager::add(MessageHandler * handler, bool allow_replace)
{
  int idx = -1;
  bool rtn = false;

  if (NULL != handler)
  {
    // If get handler returns -1 then a hander for the message type
    // does not exist, and a new one should be added
    idx = getHandlerIdx(handler->getMsgType());
    if (0 > idx)
    {
      if (this->getMaxNumHandlers() > this->getNumHandlers())
      {
        this->handlers_[this->getNumHandlers()] = handler;
        this->setNumHandlers(this->getNumHandlers() + 1);
        //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Added message handler for message type: %d", handler->getMsgType());
        rtn = true;
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Max number of handlers exceeded");
        rtn = false;
      }
    }
    else if (allow_replace)
    {
      this->handlers_[idx] = handler;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Failed to add handler for: %d, handler already exists", handler->getMsgType());
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL handler not added");
    rtn = false;
  }
  return rtn;
}

MessageHandler* MessageManager::getHandler(int msg_type)
{
  int idx = getHandlerIdx(msg_type);

  if (idx < 0)
    return NULL;
  else
    return this->handlers_[idx];
}

int MessageManager::getHandlerIdx(int msg_type)
{
  int rtn = -1;
  MessageHandler* temp = NULL;

  for (unsigned int i = 0; i < this->getMaxNumHandlers(); i++)
  {
    temp = this->handlers_[i];
    if (NULL == temp) break;  // end of handler-list

    if (temp->getMsgType() == msg_type)
    {
        rtn = i;
        break;
    }
  }

  return rtn;
}

} // namespace message_manager
} // namespace industrial
