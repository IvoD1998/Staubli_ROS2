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
#include "simple_message/socket/tcp_client.hpp"
#else
#include "tcp_client.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include <netdb.h>

namespace industrial
{
namespace tcp_client
{

TcpClient::TcpClient()
{

}

TcpClient::~TcpClient()
{
  //RCLCPP_DEBUG(rclcpp::get_logger("tcp_client"), "Destructing TCPClient");
}

bool TcpClient::init(char *buff, int port_num)
{
  addrinfo *result;
  addrinfo hints = {};

  if (!connectSocketHandle())
  {
    return false;
  }

  // Initialize address data structure
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
  this->sockaddr_.sin_family = AF_INET;


  // Check for 'buff' as hostname, and use that, otherwise assume IP address
  hints.ai_family = AF_INET;  // Allow IPv4
  hints.ai_socktype = SOCK_STREAM;  // TCP socket
  hints.ai_flags = 0;  // No flags
  hints.ai_protocol = 0;  // Any protocol
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;
  if (0 == GETADDRINFO(buff, NULL, &hints, &result))
  {
    this->sockaddr_ = *((sockaddr_in *)result->ai_addr);
  }
  else
  {
    this->sockaddr_.sin_addr.s_addr = INET_ADDR(buff);
  }
  this->sockaddr_.sin_port = HTONS(port_num);

  return true;
}

bool TcpClient::makeConnect()
{
  if (isConnected())
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_client"), "Tried to connect when socket already in connected state");
    return false;
  }

  if (!connectSocketHandle())
  {
    // Logging handled by connectSocketHandle()
    return false;
  }

  int rc = CONNECT(this->getSockHandle(), (sockaddr *)&sockaddr_, sizeof(sockaddr_));
  if (SOCKET_FAIL == rc)
  {
    logSocketError("Failed to connect to server", rc, errno);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("tcp_client"), "Connected to server");
  setConnected(true);

  return true;
}

bool TcpClient::connectSocketHandle()
{
  if (isConnected())
  {
    // Already connected, nothing to do
    return true;
  }

  int sock_handle = getSockHandle();

  if (sock_handle != SOCKET_FAIL)
  {
    // Handle is stale close old handle
    CLOSE(sock_handle);
  }

  sock_handle = SOCKET(AF_INET, SOCK_STREAM, 0);
  setSockHandle(sock_handle);
  if (SOCKET_FAIL == sock_handle)
  {
    //RCLCPP_ERROR(rclcpp::get_logger("tcp_client"), "Failed to create socket");
    return false;
  }

  int disableNodeDelay = 1;
  // The set no delay disables the NAGEL algorithm
  if (SOCKET_FAIL == SET_NO_DELAY(sock_handle, disableNodeDelay))
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_client"), "Failed to set no socket delay, sending data can be delayed by up to 250ms");
  }
  return true;
}
} //tcp_client
} //industrial

