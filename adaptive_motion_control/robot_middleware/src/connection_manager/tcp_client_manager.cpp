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

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include "robot_middleware/connection_manager/tcp_client_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>

using namespace industrial::tcp_client;

namespace robot_middleware
{
namespace connection_manager
{

TcpClientManager::TcpClientManager(const std::string& name, const std::string& ip_address, int port)
  : SimpleSocketManager(name, port), ip_address_(strdup(ip_address.c_str()))
{
}

TcpClientManager::~TcpClientManager()
{
#ifndef NDEBUG
  printf("DEBUG: Destructing TcpClientManager '%s'\n", getName());
#endif
  free(ip_address_);
}

bool TcpClientManager::init()
{
  auto tcp_client = std::make_unique<TcpClient>();
  if (!tcp_client->init(ip_address_, port_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("tcp_client_manager"), "Failed to initialize connection '%s' (%s:%d)", getName(), ip_address_, port_);
    return false;
  }

  conn_ = std::move(tcp_client);
  RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "Initialized connection '%s' (%s:%d)", getName(), ip_address_, port_);

  return true;
}

bool TcpClientManager::connect()
{
  using namespace std::chrono_literals;

  if (conn_->isConnected())
  {
    RCLCPP_WARN(rclcpp::get_logger("tcp_client_manager"), "[%s] Attempted to connect while in connected state", getName());
    return true;
  }

  if (connected_once_)
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to re-establish connection", getName());
    init();
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to connect", getName());
  }

  while (rclcpp::ok())
  {
    if (conn_->makeConnect())
    {
      std::this_thread::sleep_for(250ms);
      if (conn_->isConnected())
      {
        connected_once_ = true;
        RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Connected", getName());
        return true;
      }
    }

    // throttle loop if connection fails
    int seconds = CONNECT_RETRY_DELAY;
    while (rclcpp::ok() && seconds > 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to connect again in %d seconds", getName(), seconds);
      std::this_thread::sleep_for(1s);
      seconds--;
    }
  }

  return false;
}

}  // namespace connection_manager
}  // namespace robot_middleware