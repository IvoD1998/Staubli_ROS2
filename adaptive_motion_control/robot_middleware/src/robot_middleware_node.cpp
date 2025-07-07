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

#include "robot_middleware/robot_middleware.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace robot_middleware;

int main(int argc, char** argv)
{
#ifndef NDEBUG
  RCLCPP_WARN(rclcpp::get_logger("robot_middleware_node"), "Running in DEBUG mode. The output might contain many debug messages!");
#endif

  rclcpp::init(argc, argv);
  std::shared_ptr<RobotMiddleware> middleware = std::make_shared<RobotMiddleware>();

  if (!middleware->init())
  {
    RCLCPP_ERROR(rclcpp::get_logger("robot_middleware_node"), "Could not initialize the middleware");
    return 1;
  }
  middleware->run();
  rclcpp::spin(middleware);
  rclcpp::shutdown();

  return 0;
}
