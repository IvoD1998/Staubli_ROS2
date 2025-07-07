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
#include "simple_message/socket/tcp_server.hpp"
#else
#include "tcp_server.h"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace tcp_server
{

TcpServer::TcpServer()
{
  this->setSockHandle(this->SOCKET_FAIL);
  this->setSrvrHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
}

TcpServer::~TcpServer()
{
  CLOSE(this->getSockHandle());
  CLOSE(this->getSrvrHandle());
}

bool TcpServer::init(int port_num)
{
  int rc;
  bool rtn;
  const int reuse_addr = 1;
  //int err;
  SOCKLEN_T addrSize = 0;

  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSrvrHandle(rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket created, rc: %d", rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket handle: %d", this->getSrvrHandle());

    
    SET_REUSE_ADDR(this->getSrvrHandle(), reuse_addr);

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = HTONS(port_num);

    addrSize = sizeof(this->sockaddr_);
    rc = BIND(this->getSrvrHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      RCLCPP_INFO(rclcpp::get_logger("tcp_server"), "Server socket successfully initialized");

      rc = LISTEN(this->getSrvrHandle(), 1);

      if (this->SOCKET_FAIL != rc)
      {
       RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket in listen mode");
        rtn = true;
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to set socket to listen");
        rtn = false;
      }
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to bind socket, rc: %d", rc);
      CLOSE(this->getSrvrHandle());
      rtn = false;
    }

  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to create socket, rc: %d", rc);
    rtn = false;
  }

  return rtn;
}

bool TcpServer::makeConnect()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  //int socket = this->SOCKET_FAIL;
  int disableNodeDelay = 1;
  // int err = 0;

  if (!this->isConnected())
  {
    this->setConnected(false);
    if (this->SOCKET_FAIL != this->getSockHandle())
    {
      CLOSE(this->getSockHandle());
      this->setSockHandle(this->SOCKET_FAIL);
    }

    rc = ACCEPT(this->getSrvrHandle(), NULL, NULL);

    if (this->SOCKET_FAIL != rc)
    {
      this->setSockHandle(rc);
      //RCLCPP_INFO(rclcpp::get_logger("tcp_server"), "Client socket accepted");

      // The set no delay disables the NAGEL algorithm
      rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
      // err = errno;
      // if (this->SOCKET_FAIL == rc)
      // {
        //RCLCPP_WARN(rclcpp::get_logger("tcp_server"), "Failed to set no socket delay, errno: %d, sending data can be delayed by up to 250ms", err);
      // }
      this->setConnected(true);
      rtn = true;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to accept for client connection");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_server"), "Tried to connect when socket already in connected state");
  }

  return rtn;

}

} //tcp_socket
} //industrial

