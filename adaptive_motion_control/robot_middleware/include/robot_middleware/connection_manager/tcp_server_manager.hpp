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

#pragma once

#include "robot_middleware/connection_manager/simple_socket_manager.hpp"

#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/socket/tcp_server.hpp"

#include <memory>
#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
namespace robot_middleware
{
namespace connection_manager
{

class TcpServerManager : public SimpleSocketManager
{
public:
  TcpServerManager(const std::string& name, int port);

  ~TcpServerManager() override;

  bool init() override;

  bool connect() override;
};

using TcpServerManagerPtr = std::shared_ptr<TcpServerManager>;

}  // namespace connection_manager
}  // namespace robot_middleware