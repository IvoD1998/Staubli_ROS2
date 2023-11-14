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

#include "robot_middleware/jog_interface.hpp"
#include "robot_middleware/message_relay_handler/motion_relay_handler.hpp"
#include "robot_middleware/message_relay_handler/state_relay_handler.hpp"
#include "robot_middleware/pose_tracking_controller.hpp"
#include "robot_middleware/robot_driver.hpp"
#include "robot_middleware/robot_server_proxy.hpp"
#include "robot_middleware/staubli/staubli_driver.hpp"
#include "robot_middleware/velocity_control_settings.hpp"

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_message/socket/tcp_server.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace robot_middleware
{

class RobotMiddleware : public rclcpp::Node
{
public:
  explicit RobotMiddleware();

  ~RobotMiddleware();

  bool init();

  void run();

private:

  std::shared_ptr<RobotDriver> driver_;
  std::shared_ptr<RobotServerProxy> server_proxy_;
  std::shared_ptr<JogInterface> jog_interface_;
  std::shared_ptr<PoseTrackingController> pose_tracking_controller_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  std::shared_ptr<robot_middleware::message_relay_handler::StateRelayHandler> state_relay_handler_;
  std::shared_ptr<robot_middleware::message_relay_handler::MotionRelayHandler> motion_relay_handler_;
};

}  // namespace robot_middleware