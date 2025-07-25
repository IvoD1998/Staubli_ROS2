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
#include "simple_message/socket/tcp_socket.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "tcp_socket.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace tcp_socket
{

TcpSocket::TcpSocket()
{
}

TcpSocket::~TcpSocket()
// Closes socket
{
  //RCLCPP_DEBUG(rclcpp::get_logger("tcp_socket"), "Destructing TCPSocket");
  CLOSE(this->getSockHandle());
}

int TcpSocket::rawSendBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;

  rc = SEND(this->getSockHandle(), buffer, num_bytes, 0);
  
  return rc;
}

int TcpSocket::rawReceiveBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  
  rc = RECV(this->getSockHandle(), buffer, num_bytes, 0);
  
  return rc;
}

bool TcpSocket::rawPoll(int timeout, bool & ready, bool & error)
{
  timeval time;
  fd_set read, write, except;
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  ready = false;
  error = false;

  // The select function uses the timeval data structure
  time.tv_sec = timeout / 1000;
  time.tv_usec = (timeout % 1000) * 1000;

  FD_ZERO(&read);
  FD_ZERO(&write);
  FD_ZERO(&except);

  FD_SET(this->getSockHandle(), &read);
  FD_SET(this->getSockHandle(), &except);

  rc = SELECT(this->getSockHandle() + 1, &read, &write, &except, &time);

  if (this->SOCKET_FAIL != rc) {
    if (0 == rc)
      rtn = false;
    else {
      if (FD_ISSET(this->getSockHandle(), &read)) {
        ready = true;
        rtn = true;
      }
      else if(FD_ISSET(this->getSockHandle(), &except)) {
        error = true;
        rtn = true;
      }
      else {
        //RCLCPP_WARN(rclcpp::get_logger("tcp_socket"), "Select returned, but no flags are set");
        rtn = false;
      }
    }
  } else {
    this->logSocketError("Socket select function failed", rc, errno);
    rtn = false;
  }
  return rtn;
}

} //tcp_socket
} //industrial

