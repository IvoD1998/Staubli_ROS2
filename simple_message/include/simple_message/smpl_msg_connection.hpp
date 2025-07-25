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

#pragma once

#ifndef FLATHEADERS
#include "simple_message/byte_array.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "byte_array.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#endif


namespace industrial
{
namespace smpl_msg_connection
{

/**
 * \brief Defines an interface and common methods for sending simple messages 
 * (see simple_message).  This interface makes a bare minimum of assumptions:
 *
 * 1. The connection is capable of sending raw bytes (encapsulated within a simple message)
 *
 * 2. The data connection has an explicit connect that establishes the connection (and an 
 *    associated disconnect method).  NOTE: For data connections that are connectionless,
 *    such as UDP, the connection method can be a NULL operation.
 */
class SmplMsgConnection

{
public:

  // Message

  /**
   * \brief Sends a message using the data connection
   *
   * \param message to send
   *
   * \return true if successful
   */
  virtual bool sendMsg(industrial::simple_message::SimpleMessage & message);

  /**
   * \brief Receives a message using the data connection
   *
   * \param populated with received message
   *
   * \return true if successful
   */
  virtual bool receiveMsg(industrial::simple_message::SimpleMessage & message);

  /**
   * \brief Receives a message using the data connection with a timeout.
   *
   * \param [out] message Populated with received message
   * \param [in] timeout_ms The timeout for receiving a message, in milliseconds
   *
   * \return true if successful
   */
  virtual bool receiveMsg(industrial::simple_message::SimpleMessage & message,
                          industrial::shared_types::shared_int timeout_ms);

  /**
   * \brief Performs a complete send and receive.  This is helpful when sending
   * a message that requires and explicit reply
   *
   * \param message to send
   * \param populated with received message
   * \param verbosity level of low level logging
   *
   * \return true if successful
   */
  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage & send,
                         industrial::simple_message::SimpleMessage & recv,
                         bool verbose = false);

  /**
   * \brief Performs a complete send and receive with a timeout.
   * This is helpful when sending a message that requires and explicit reply.
   *
   * \param [in] send The message to send
   * \param [out] recv Populated with received message
   * \param [in] timeout_ms The timeout for receiving a message, in milliseconds
   * \param [in] verbose Turn on low level logging
   *
   * \return true if successful
   */
  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage & send,
                         industrial::simple_message::SimpleMessage & recv,
                         industrial::shared_types::shared_int timeout_ms,
                         bool verbose = false);

  /**
   * \brief return connection status
   *
   * \return true if connected
   */
  virtual bool isConnected()=0;

  /**
   * \brief connects to the remote host
   *
   * \return true on success, false otherwise
   */
  virtual bool makeConnect()=0;

private:

  // Overrides
  /**
   * \brief Method used by send message interface method.  This should be overridden 
   * for the specific connection type
   *
   * \param data to send.
   *
   * \return true if successful
   */
  virtual bool sendBytes(industrial::byte_array::ByteArray & buffer) =0;

  /**
   * \brief Method used by receive message interface method.  This should be overridden
   * for the specific connection type.
   *
   * \param data to receive.
   * \param size (in bytes) of data to receive
   * \param timeout_ms Timeout to receive a message (in milliseconds). A negative timeout
   * means that this function should wait indefinitely.
   *
   * \return true if successful
   */
  virtual bool receiveBytes(industrial::byte_array::ByteArray & buffer,
                            industrial::shared_types::shared_int num_bytes,
                            industrial::shared_types::shared_int timeout_ms) = 0;
};

} //namespace message_connection
} //namespace industrial
