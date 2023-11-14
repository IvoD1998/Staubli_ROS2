
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
#include "staubli_val3_driver/joint_feedback_relay_handler.hpp"
#include "industrial_robot_client/robot_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  // initialize node
  rclcpp::init(argc, argv);

  // launch the default Robot State Interface connection/handlers
  std::shared_ptr<industrial_robot_client::robot_state_interface::RobotStateInterface> rsi = std::make_shared<industrial_robot_client::robot_state_interface::RobotStateInterface>();
  
  rsi->init();

  // add the JointFeedback handler
  JointFeedbackRelayHandler joint_fbk_handler;
  std::vector<std::string> joint_names = rsi->get_joint_names();
  joint_fbk_handler.init(rsi->get_connection(), joint_names);
  rsi->add_handler(&joint_fbk_handler);
  // run the node
  rsi->run();

  rclcpp::spin(rsi);
  rclcpp::shutdown();
  return 0;
}