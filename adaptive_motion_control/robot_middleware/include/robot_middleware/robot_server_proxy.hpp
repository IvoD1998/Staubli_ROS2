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

#include "robot_middleware/connection_manager/tcp_server_manager.hpp"

namespace robot_middleware
{

class RobotServerProxy
{
public:
  RobotServerProxy(std::shared_ptr<rclcpp::Node> node);

  ~RobotServerProxy();

  bool init(int default_motion_port, int default_state_port);

  const robot_middleware::connection_manager::SimpleSocketManagerPtr& getMotionServerManager()
  {
    return motion_server_manager_;
  }

  const robot_middleware::connection_manager::SimpleSocketManagerPtr& getStateServerManager()
  {
    return state_server_manager_;
  }

  void connect();

private:
  std::shared_ptr<rclcpp::Node> node_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr motion_server_manager_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr state_server_manager_;
};

}  // namespace robot_middleware