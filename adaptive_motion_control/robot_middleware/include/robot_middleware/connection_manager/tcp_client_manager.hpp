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

#pragma once

#include "robot_middleware/connection_manager/simple_socket_manager.hpp"

#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/socket/tcp_client.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
namespace robot_middleware
{
namespace connection_manager
{

class TcpClientManager : public SimpleSocketManager
{
public:
  TcpClientManager(const std::string& name, const std::string& ip_address, int port);

  ~TcpClientManager() override;

  bool init() override;

  bool connect() override;

private:
  // sleep time before retrying to connect in seconds
  const int CONNECT_RETRY_DELAY = 3;

  char* ip_address_;
};

using TcpClientManagerPtr = std::shared_ptr<TcpClientManager>;

}  // namespace connection_manager
}  // namespace robot_middleware