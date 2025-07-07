/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Modified from original version in https://github.com/ros-industrial/industrial_core
// Changes made to support ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#include "industrial_robot_client/robot_state_interface.hpp"
#include "industrial_utils/param_utils.hpp"

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::ParamUtils;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace robot_state_interface
{

RobotStateInterface::RobotStateInterface() : Node("robot_state_interface")
{ 
  this->connection_ = NULL;
  this->add_handler(&default_joint_handler_);
  this->add_handler(&default_robot_status_handler_);
}

bool RobotStateInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->declare_parameter<int>("~port", default_port);
  this->get_parameter("robot_ip_address", ip);
  this->get_parameter("~port", port);

  // check for valid parameter values
  if (ip.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  RCLCPP_INFO(this->get_logger(), "Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool RobotStateInterface::init(industrial::smpl_msg_connection::SmplMsgConnection *connection)
{
  std::vector<std::string> joint_names;
  ParamUtils pu;
  // if (!pu.getJointNames("move_group", "rviz2", "controller_joint_names", "robot_description", joint_names))
  if (!pu.getJointNames("move_group", "moveit_simple_controller_manager.manipulator_controller.joints", joint_names))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool RobotStateInterface::init(industrial::smpl_msg_connection::SmplMsgConnection *connection, 
                               std::vector<std::string>& joint_names)
{
  this->joint_names_ = joint_names;
  this->connection_ = connection;
  connection_->makeConnect();

  // initialize message-manager
  if (!manager_.init(connection_))
    return false;

  // initialize default handlers
  if (!default_joint_handler_.init(connection_, joint_names_))
    return false;
  this->add_handler(&default_joint_handler_);

  if (!default_robot_status_handler_.init(connection_))
      return false;
  this->add_handler(&default_robot_status_handler_);

  return true;
}

void RobotStateInterface::run()
{
  manager_.spin();
}

} // robot_state_interface
} // industrial_robot_client