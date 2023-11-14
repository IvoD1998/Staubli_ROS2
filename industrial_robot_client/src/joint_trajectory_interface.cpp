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

#include <algorithm>
#include "industrial_robot_client/joint_trajectory_interface.hpp"
#include "simple_message/joint_traj_pt.hpp"
#include "industrial_utils/param_utils.hpp"
#include "rcpputils/asserts.hpp"


using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;
typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::msg::JointTrajectoryPoint  ros_JointTrajPt;

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

JointTrajectoryInterface::JointTrajectoryInterface() : Node("joint_trajectory_interface"), default_joint_pos_(0.0), default_vel_ratio_(0.1), default_duration_(10.0)
{
}

bool JointTrajectoryInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->get_parameter<std::string>("robot_ip_address", ip);
  this->declare_parameter<int> ("~port", default_port);
  this->get_parameter<int>("~port", port);

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
  RCLCPP_INFO(this->get_logger(), "Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection)
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

bool JointTrajectoryInterface::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                    const std::map<std::string, double> &velocity_limits)
{
  using namespace std::placeholders;

  this->connection_ = connection;
  this->all_joint_names_ = joint_names;
  this->joint_vel_limits_ = velocity_limits;
  connection_->makeConnect();

  // try to read velocity limits from URDF, if none specified
  ParamUtils pu;
  if (joint_vel_limits_.empty() && !pu.getJointVelocityLimits("rviz2", "robot_description", joint_vel_limits_))
    RCLCPP_WARN(this->get_logger(), "Unable to read velocity limits from 'robot_description' param.  Velocity validation disabled.");



  this->srv_stop_motion_ = this->create_service<industrial_msgs::srv::StopMotion>("stop_motion", std::bind(&JointTrajectoryInterface::stopMotionCB, this, _1, _2));
  this->srv_joint_trajectory_ = this->create_service<industrial_msgs::srv::CmdJointTrajectory>("joint_path_command", std::bind(&JointTrajectoryInterface::jointTrajectoryCB, this, _1, _2));
  this->sub_joint_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 1, std::bind(&JointTrajectoryInterface::jointTrajectorySubCB, this, _1));
  this->sub_cur_pos_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&JointTrajectoryInterface::jointStateCB, this, _1));

  return true;
}

JointTrajectoryInterface::~JointTrajectoryInterface()
{  
  trajectoryStop();
}

void JointTrajectoryInterface::jointTrajectoryCB(const std::shared_ptr<industrial_msgs::srv::CmdJointTrajectory::Request> req,
                                                 std::shared_ptr<industrial_msgs::srv::CmdJointTrajectory::Response> res)
{
  trajectory_msgs::msg::JointTrajectory::Ptr traj_ptr(new trajectory_msgs::msg::JointTrajectory);
  *traj_ptr = req->trajectory;  // copy message data
  this->jointTrajectorySubCB(traj_ptr);

  // no success/fail result from jointTrajectoryCB.  Assume success.
  res->code.val = industrial_msgs::msg::ServiceReturnCode::SUCCESS;

}

void JointTrajectoryInterface::jointTrajectorySubCB(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Receiving joint trajectory message");

  // check for STOP command
  if (msg->points.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return;
  }

  // convert trajectory into robot-format
  std::vector<JointTrajPtMessage> robot_msgs;
  if (!trajectory_to_msgs(msg, &robot_msgs))
  {
    RCLCPP_ERROR(this->get_logger(), "Trajectory not processed");
    return;
  }
  // send command messages to robot
  send_to_robot(robot_msgs);
  RCLCPP_INFO(this->get_logger(), "Trajectory message sent");
}

bool JointTrajectoryInterface::trajectory_to_msgs(const trajectory_msgs::msg::JointTrajectory::SharedPtr &traj, std::vector<JointTrajPtMessage>* msgs)
{
  msgs->clear();
  // check for valid trajectory
  if (!is_valid(*traj))
  {
    RCLCPP_ERROR(this->get_logger(), "Given trajectory not valid...");
    return false;
  }

  for (size_t i=0; i<traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // reduce velocity to a single scalar, for robot command
    if (!calc_speed(xform_pt, &vel, &duration))
      return false;

    JointTrajPtMessage msg = create_message(i, xform_pt.positions, vel, duration);

    msgs->push_back(msg);
  }

  return true;
}

bool JointTrajectoryInterface::select(const std::vector<std::string>& ros_joint_names, const ros_JointTrajPt& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, ros_JointTrajPt* rbt_pt)
{
  rcpputils::assert_true(ros_joint_names.size() == ros_pt.positions.size());

  // initialize rbt_pt
  *rbt_pt = ros_pt;
  rbt_pt->positions.clear(); rbt_pt->velocities.clear(); rbt_pt->accelerations.clear();

  for (size_t rbt_idx=0; rbt_idx < rbt_joint_names.size(); ++rbt_idx)
  {
    bool is_empty = rbt_joint_names[rbt_idx].empty();

    // find matching ROS element
    size_t ros_idx = std::find(ros_joint_names.begin(), ros_joint_names.end(), rbt_joint_names[rbt_idx]) - ros_joint_names.begin();
    bool is_found = ros_idx < ros_joint_names.size();

    // error-chk: required robot joint not found in ROS joint-list
    if (!is_empty && !is_found)
    {
      RCLCPP_ERROR(this->get_logger(), "Expected joint (%s) not found in JointTrajectory.  Aborting command.", rbt_joint_names[rbt_idx].c_str());
      return false;
    }

    if (is_empty)
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(default_joint_pos_);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(-1);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(-1);
    }
    else
    {
      if (!ros_pt.positions.empty()) rbt_pt->positions.push_back(ros_pt.positions[ros_idx]);
      if (!ros_pt.velocities.empty()) rbt_pt->velocities.push_back(ros_pt.velocities[ros_idx]);
      if (!ros_pt.accelerations.empty()) rbt_pt->accelerations.push_back(ros_pt.accelerations[ros_idx]);
    }
  }
  return true;
}

bool JointTrajectoryInterface::calc_speed(const trajectory_msgs::msg::JointTrajectoryPoint& pt, double* rbt_velocity, double* rbt_duration)
{
	return calc_velocity(pt, rbt_velocity) && calc_duration(pt, rbt_duration);
}

// default velocity calculation computes the %-of-max-velocity for the "critical joint" (closest to velocity-limit)
// such that 0.2 = 20% of maximum joint speed.
//
// NOTE: this calculation uses the maximum joint speeds from the URDF file, which may differ from those defined on
// the physical robot.  These differences could lead to different actual movement velocities than intended.
// Behavior should be verified on a physical robot if movement velocity is critical.
bool JointTrajectoryInterface::calc_velocity(const trajectory_msgs::msg::JointTrajectoryPoint& pt, double* rbt_velocity)
{
  std::vector<double> vel_ratios;

  rcpputils::assert_true(all_joint_names_.size() == pt.positions.size());

  // check for empty velocities in ROS topic
  if (pt.velocities.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Joint velocities unspecified.  Using default/safe speed.");
    *rbt_velocity = default_vel_ratio_;
    return true;
  }

  for (size_t i=0; i<all_joint_names_.size(); ++i)
  {
    const std::string &jnt_name = all_joint_names_[i];

    // update vel_ratios
    if (jnt_name.empty())                             // ignore "dummy joints" in velocity calcs
      vel_ratios.push_back(-1);
    else if (joint_vel_limits_.count(jnt_name) == 0)  // no velocity limit specified for this joint
      vel_ratios.push_back(-1);
    else
      vel_ratios.push_back( fabs(pt.velocities[i] / joint_vel_limits_[jnt_name]) );  // calculate expected duration for this joint
  }

  // find largest velocity-ratio (closest to max joint-speed)
  int max_idx = std::max_element(vel_ratios.begin(), vel_ratios.end()) - vel_ratios.begin();
  
  if (vel_ratios[max_idx] > 0)
    *rbt_velocity = vel_ratios[max_idx];
  else
  {
    RCLCPP_WARN_ONCE(this->get_logger(), "Joint velocity-limits unspecified.  Using default velocity-ratio.");
    *rbt_velocity = default_vel_ratio_;
  }

  if ( (*rbt_velocity < 0) || (*rbt_velocity > 1) )
  {
    RCLCPP_WARN(this->get_logger(), "computed velocity (%.1f %%) is out-of-range.  Clipping to [0-100%%]", *rbt_velocity * 100);
    *rbt_velocity = std::min(1.0, std::max(0.0, *rbt_velocity));  // clip to [0,1]
  }
  
  return true;
}

bool JointTrajectoryInterface::calc_duration(const trajectory_msgs::msg::JointTrajectoryPoint& pt, double* rbt_duration)
{
  std::vector<double> durations;
  double this_time = pt.time_from_start.sec;
  static double last_time = 0;

  if (this_time <= last_time)  // earlier time => new trajectory.  Move slowly to first point.
    *rbt_duration = default_duration_;
  else
    *rbt_duration = this_time - last_time;

  last_time = this_time;

  return true;
}

JointTrajPtMessage JointTrajectoryInterface::create_message(int seq, std::vector<double> joint_pos, double velocity, double duration)
{
  industrial::joint_data::JointData pos;
  rcpputils::assert_true(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  for (size_t i=0; i<joint_pos.size(); ++i)
    pos.setJoint(i, joint_pos[i]);

  rbt_JointTrajPt pt;
  pt.init(seq, pos, velocity, duration);

  JointTrajPtMessage msg;
  msg.init(pt);

  return msg;
}

void JointTrajectoryInterface::trajectoryStop()
{
  JointTrajPtMessage jMsg;
  SimpleMessage msg, reply;

  RCLCPP_INFO(this->get_logger(), "Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  RCLCPP_DEBUG(this->get_logger(), "Sending stop command");
  this->connection_->sendAndReceiveMsg(msg, reply);
}

void JointTrajectoryInterface::stopMotionCB(const std::shared_ptr<industrial_msgs::srv::StopMotion::Request> req,
                                    std::shared_ptr<industrial_msgs::srv::StopMotion::Response> res)
{
  trajectoryStop();

  // no success/fail result from trajectoryStop.  Assume success.
  res->code.val = industrial_msgs::msg::ServiceReturnCode::SUCCESS;

}

bool JointTrajectoryInterface::is_valid(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (int i=0; i<traj.points.size(); ++i)
  {
    const trajectory_msgs::msg::JointTrajectoryPoint &pt = traj.points[i];

    // check for non-empty positions
    if (pt.positions.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Validation failed: Missing position data for trajectory pt %d", i);
      return false;
    }

    // check for joint velocity limits
    for (int j=0; j<pt.velocities.size(); ++j)
    {
      std::map<std::string, double>::iterator max_vel = joint_vel_limits_.find(traj.joint_names[j]);
      if (max_vel == joint_vel_limits_.end()) 
      {
        continue;
      }

      if (std::abs(pt.velocities[j]) > max_vel->second)
        RCLCPP_ERROR(this->get_logger(), "Validation failed: Max velocity exceeded for trajectory pt %d, joint '%s'", i, traj.joint_names[j].c_str());
        return false;
    }
    // check for valid timestamp
    if ((i > 0) && ((pt.time_from_start.sec == 0) && (pt.time_from_start.nanosec == 0)))
    {
      RCLCPP_ERROR(this->get_logger(), "Validation failed: Missing valid timestamp data for trajectory pt %d", i);
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Given trajectory is valid");
  return true;
}


// copy robot JointState into local cache
void JointTrajectoryInterface::jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  this->cur_joint_pos_ = *msg;
}

} //joint_trajectory_interface
} //industrial_robot_client

