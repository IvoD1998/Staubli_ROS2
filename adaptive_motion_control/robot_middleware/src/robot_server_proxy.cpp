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

#include "robot_middleware/robot_server_proxy.hpp"

using namespace robot_middleware::connection_manager;

namespace robot_middleware
{

RobotServerProxy::RobotServerProxy(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
}

RobotServerProxy::~RobotServerProxy()
{
}

bool RobotServerProxy::init(int default_motion_port, int default_state_port)
{
  int motion_port, state_port;

  node_->declare_parameter<int>("robot_server_proxy_motion_server_port", default_motion_port);
  node_->declare_parameter<int>("robot_server_proxy_state_server_port", default_state_port);
  node_->get_parameter<int>("robot_server_proxy_motion_server_port", motion_port);
  node_->get_parameter<int>("robot_server_proxy_state_server_port", state_port);

  RCLCPP_INFO(node_->get_logger(), "Using motion server port: %d", motion_port);
  RCLCPP_INFO(node_->get_logger(), "Using state server port: %d", state_port);

  auto motion_server_manager = std::make_shared<TcpServerManager>("motion_server", motion_port);
  auto state_server_manager = std::make_shared<TcpServerManager>("state_server", state_port);

  if (!motion_server_manager->init())
    return false;

  if (!state_server_manager->init())
    return false;

  motion_server_manager_ = std::move(motion_server_manager);
  state_server_manager_ = std::move(state_server_manager);

  return true;
}

void RobotServerProxy::connect()
{
  motion_server_manager_->startConnectionTask();
  state_server_manager_->startConnectionTask();
}

}  // namespace robot_middleware
