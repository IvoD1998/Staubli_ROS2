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

#include "simple_message/simple_message.hpp"
#include "simple_message/socket/simple_socket.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
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

class SimpleSocketManager
{
public:
  const char* getName();

  void setFriendConnection(const std::shared_ptr<SimpleSocketManager>& conn);

  bool isConnected();

  bool isReadyReceive(int timeout);

  virtual bool init() = 0;

  virtual bool connect() = 0;

  virtual void disconnect();

  void startConnectionTask();

  bool sendMsg(industrial::simple_message::SimpleMessage& msg);

  bool receiveMsg(industrial::simple_message::SimpleMessage& msg);

  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage& send,
                         industrial::simple_message::SimpleMessage& recv);

protected:
  SimpleSocketManager(const std::string& name, int port);

  virtual ~SimpleSocketManager();

  const std::string LOGNAME = "connection_manager";
  const std::string name_;
  int port_;
  bool connected_once_;
  std::unique_ptr<industrial::simple_socket::SimpleSocket> conn_;

private:
  void connectionTask();

  std::mutex connection_mtx_;
  std::condition_variable connection_lost_cv_;
  std::shared_ptr<SimpleSocketManager> friend_connection_;
};

using SimpleSocketManagerPtr = std::shared_ptr<SimpleSocketManager>;

}  // namespace connection_manager
}  // namespace robot_middleware