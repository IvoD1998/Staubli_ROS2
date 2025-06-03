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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_control_msgs/srv/get_motion_plan.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "std_srvs/srv/trigger.hpp"

class MoveItInterface : public rclcpp::Node
{
public:
    explicit MoveItInterface();

    ~MoveItInterface();
    void init();
    void run();

private:

    void getMotionPlan(const std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Request> req,
                       std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Response> res);

    void executeMotionPlan(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    const int DEFAULT_PLANNING_TIME = 5; // in seconds
    const double DEFAULT_VELOCITY_SCALING = 0.1;
    const double DEFAULT_ACCELERATION_SCALING = 0.1;
    rclcpp::Service<motion_control_msgs::srv::GetMotionPlan>::SharedPtr plan_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;
};