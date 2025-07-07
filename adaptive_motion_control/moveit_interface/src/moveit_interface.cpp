/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Institute for Factory Automation and Production Systems (FAPS)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include "moveit_interface/moveit_interface.hpp"

#include <exception>
#include <memory>
#include <string>

using namespace moveit::planning_interface;
using namespace std::placeholders;
using namespace std::chrono_literals;

MoveItInterface::MoveItInterface() : Node("moveit_interface")
{
}

MoveItInterface::~MoveItInterface()
{
}

void MoveItInterface::init()
{
  RCLCPP_INFO(this->get_logger(), "Initializing the MoveIt interface");
  std::string planning_group;
  
  //Setup the planning group the 
  this->declare_parameter<std::string>("planning_group", "manipulator");
  if (!this->get_parameter("planning_group", planning_group))
  {
    throw std::runtime_error("Missing required parameter 'planning_group'");
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Using planning_group: %s", planning_group.c_str());
  }
  
  //We need to get the robot description as a parameter of this node to initialise the robot_model_loader properly.
  std::vector <rclcpp::Parameter> xml_strings = {};
  rclcpp::Parameter xml_string;
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  xml_strings = parameters_client->get_parameters({"robot_description"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("robot_description", xml_string.as_string());
  //Do the same for the semantic description
  xml_strings = {};
  parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  xml_strings = parameters_client->get_parameters({"robot_description_semantic"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("robot_description_semantic", xml_string.as_string());

  //Construct the move_group pointer
  move_group_ = std::make_shared<MoveGroupInterface>(this->shared_from_this(), planning_group);
  plan_ = std::make_shared<MoveGroupInterface::Plan>();

  // print info
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());

  // init service servers for planning and execution
  plan_service_ = this->create_service<motion_control_msgs::srv::GetMotionPlan>("plan", std::bind(&MoveItInterface::getMotionPlan, this, _1, _2));
  execute_service_ = this->create_service<std_srvs::srv::Trigger>("execute", std::bind(&MoveItInterface::executeMotionPlan, this, _1, _2));
}

void MoveItInterface::run()
{
  // wait for /joint_states topic
  RCLCPP_INFO(this->get_logger(), "Waiting for a message from the topic /joint_states");
  RCLCPP_INFO(this->get_logger(), "Ready to receive motion requests");
}

void MoveItInterface::getMotionPlan(const std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Request> req,
                                    std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Received 'plan' request.");

  // planner id
  if (req->planner_id.empty())
    move_group_->setPlannerId(move_group_->getDefaultPlannerId());
  else
    move_group_->setPlannerId(req->planner_id);

  // planning time
  if (req->allowed_planning_time <= 0)
    move_group_->setPlanningTime(DEFAULT_PLANNING_TIME);
  else
    move_group_->setPlanningTime(req->allowed_planning_time);

  // velocity scaling
  double velocity_scaling =
      (req->max_velocity_scaling_factor <= 0) ? DEFAULT_VELOCITY_SCALING : req->max_velocity_scaling_factor;
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);

  // acceleration scaling
  double acceleration_scaling =
      (req->max_acceleration_scaling_factor <= 0) ? DEFAULT_ACCELERATION_SCALING : req->max_acceleration_scaling_factor;
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

  // pose reference frame
  if (req->goal.header.frame_id.empty())
    move_group_->setPoseReferenceFrame(move_group_->getPlanningFrame());
  else
    move_group_->setPoseReferenceFrame(req->goal.header.frame_id);

  // pose goal
  move_group_->setPoseTarget(req->goal.pose);

  // clang-format off
  RCLCPP_INFO_STREAM(this->get_logger(), 
                  "Planning parameters:\n"
                  "  Planner ID:           " << move_group_->getPlannerId() << "\n"
                  "  Planning time:        " << move_group_->getPlanningTime() << "\n"
                  "  Velocity scaling:     " << velocity_scaling << "\n"
                  "  Acceleration scaling: " << acceleration_scaling << "\n"
                  "  Pose reference frame: " << move_group_->getPoseReferenceFrame() << "\n"
                  "  Pose goal:            " << "P.xyz = [" << req->goal.pose.position.x << ", "
                                                            << req->goal.pose.position.y << ", "
                                                            << req->goal.pose.position.z << "], "
                                             << "Q.xyzw = [" << req->goal.pose.orientation.x << ", "
                                                             << req->goal.pose.orientation.y << ", "
                                                             << req->goal.pose.orientation.z << ", "
                                                             << req->goal.pose.orientation.w << "]");
  // clang-format on

  // plan
  plan_ = std::make_shared<MoveGroupInterface::Plan>();
  moveit::core::MoveItErrorCode error_code = move_group_->plan(*plan_);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Motion planning succeeded. Execution can be triggered now.");
    res->success = true;
    res->trajectory = plan_->trajectory.joint_trajectory;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),"Failed to find a motion plan.");
    res->success = false;
  }

  res->error_code = error_code;

  return;
}

void MoveItInterface::executeMotionPlan(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)*req;
  RCLCPP_INFO(this->get_logger(), "Received 'execute' request.");
  moveit::core::MoveItErrorCode error_code = move_group_->execute(*plan_);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
  {
    res->success = true;
    res->message = "Motion plan was executed successfully.";
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
  }
  else
  {
    res->success = false;
    res->message = "Motion plan could not be executed. MoveIt error code: " + std::to_string(error_code.val);
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
  }

  return;
}