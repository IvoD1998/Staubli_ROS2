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

#include "robot_middleware/pose_tracking_controller.hpp"
#include "moveit/moveit_cpp/moveit_cpp.h"

#include <cassert>
#include <cstring>

#ifndef NDEBUG
#include <cstdio>
#endif

namespace robot_middleware
{

PoseTrackingController::PoseTrackingController(std::shared_ptr<rclcpp::Node> node) : driver_(nullptr)
  , controller_period_(0, 0)
  , goal_timestamp_(0.0)
  , last_goal_timestamp_(0.0)
  , tcp_frame_(DEFAULT_TCP_FRAME)
  , goal_tolerance_linear_(DEFAULT_GOAL_TOLERANCE_LINEAR)
  , goal_tolerance_angular_(DEFAULT_GOAL_TOLERANCE_ANGULAR)
  , goal_settle_time_(DEFAULT_GOAL_SETTLE_TIME)
  , receive_timeout_(DEFAULT_RECEIVE_TIMEOUT)
  , keep_running_(false)
  , in_error_(false)
{
  node_ = node;
  //First define the PID defaults
  DEFAULT_PID_LINEAR.p = 1.0;
  DEFAULT_PID_LINEAR.i = 0.1;
  DEFAULT_PID_LINEAR.d = 0.0;
  DEFAULT_PID_LINEAR.i_clamp = 0.010; //-> corresponding to 10 mm/s

  DEFAULT_PID_ANGULAR.p = 1.0;
  DEFAULT_PID_ANGULAR.i = 0.1;
  DEFAULT_PID_ANGULAR.d = 0.0;
  DEFAULT_PID_ANGULAR.i_clamp = 10.0; //->corresponding to 10 deg/s, loaded as rad/s

}

PoseTrackingController::~PoseTrackingController()
{
  stop();
}

double PoseTrackingController::toRadians(double angle_degrees)
{
  double angle_rad = angle_degrees * (M_PI/180);
  return angle_rad;
}

void PoseTrackingController::init(const std::shared_ptr<RobotDriver>& driver)
{
  driver_ = driver;

  // set cycle time of the controller
  assert(driver_->getVelocityControlSettings().control_loop_frequency > 0);

  /////////////////////////////////////////////
  double temp = (1/driver_->getVelocityControlSettings().control_loop_frequency)*1000000000;
  std::chrono::nanoseconds temp2((int) temp);
  controller_period_ = rclcpp::Duration(temp2);
  /////////////////////////////////////////////

  // printf("control loop seconds: %f.5\n"
  //        "control loop nanoseconds: %f.5\n",
  //        controller_period_.seconds(),
  //        controller_period_.nanoseconds());

  // controller_period_ = rclcpp::Duration(1.0 / driver_->getVelocityControlSettings().control_loop_frequency, 0);

  node_->declare_parameter<std::string>("pose_tracking_tcp_frame", DEFAULT_TCP_FRAME);
  node_->declare_parameter<double>("pose_tracking_goal_tolerance_linear", DEFAULT_GOAL_TOLERANCE_LINEAR);
  node_->declare_parameter<double>("pose_tracking_goal_tolerance_angular", DEFAULT_GOAL_TOLERANCE_ANGULAR);
  node_->declare_parameter<double>("pose_tracking_goal_settle_time", DEFAULT_GOAL_SETTLE_TIME);
  node_->declare_parameter<double>("pose_tracking_receive_timeout", DEFAULT_RECEIVE_TIMEOUT);
  node_->declare_parameter<double>("pose_tracking_pid_linear_x_p", DEFAULT_PID_LINEAR.p);
  node_->declare_parameter<double>("pose_tracking_pid_linear_x_i", DEFAULT_PID_LINEAR.i);
  node_->declare_parameter<double>("pose_tracking_pid_linear_x_d", DEFAULT_PID_LINEAR.d);
  node_->declare_parameter<double>("pose_tracking_pid_linear_x_i_clamp", DEFAULT_PID_LINEAR.i_clamp);
  node_->declare_parameter<double>("pose_tracking_pid_linear_y_p", DEFAULT_PID_LINEAR.p);
  node_->declare_parameter<double>("pose_tracking_pid_linear_y_i", DEFAULT_PID_LINEAR.i);
  node_->declare_parameter<double>("pose_tracking_pid_linear_yid", DEFAULT_PID_LINEAR.d);
  node_->declare_parameter<double>("pose_tracking_pid_linear_y_i_clamp", DEFAULT_PID_LINEAR.i_clamp);
  node_->declare_parameter<double>("pose_tracking_pid_linear_z_p", DEFAULT_PID_LINEAR.p);
  node_->declare_parameter<double>("pose_tracking_pid_linear_z_i", DEFAULT_PID_LINEAR.i);
  node_->declare_parameter<double>("pose_tracking_pid_linear_zid", DEFAULT_PID_LINEAR.d);
  node_->declare_parameter<double>("pose_tracking_pid_linear_z_i_clamp", DEFAULT_PID_LINEAR.i_clamp);
  node_->declare_parameter<double>("pose_tracking_pid_angular_p", DEFAULT_PID_ANGULAR.p);
  node_->declare_parameter<double>("pose_tracking_pid_angular_i", DEFAULT_PID_ANGULAR.i);
  node_->declare_parameter<double>("pose_tracking_pid_angular_d", DEFAULT_PID_ANGULAR.d);
  node_->declare_parameter<double>("pose_tracking_pid_angular_i_clamp", DEFAULT_PID_ANGULAR.i_clamp);

  node_->get_parameter<std::string>("pose_tracking_tcp_frame", tcp_frame_);
  node_->get_parameter<double>("pose_tracking_goal_tolerance_linear", goal_tolerance_linear_);
  node_->get_parameter<double>("pose_tracking_goal_tolerance_angular", goal_tolerance_angular_);
  node_->get_parameter<double>("pose_tracking_goal_settle_time", goal_settle_time_);
  node_->get_parameter<double>("pose_tracking_receive_timeout", receive_timeout_);
  node_->get_parameter<double>("pose_tracking_pid_linear_x_p", pid_linear_x_params_.p);
  node_->get_parameter<double>("pose_tracking_pid_linear_x_i", pid_linear_x_params_.i);
  node_->get_parameter<double>("pose_tracking_pid_linear_xid", pid_linear_x_params_.d);
  node_->get_parameter<double>("pose_tracking_pid_linear_x_i_clamp", pid_linear_x_params_.i_clamp);
  node_->get_parameter<double>("pose_tracking_pid_linear_y_p", pid_linear_y_params_.p);
  node_->get_parameter<double>("pose_tracking_pid_linear_y_i", pid_linear_y_params_.i);
  node_->get_parameter<double>("pose_tracking_pid_linear_yid", pid_linear_y_params_.d);
  node_->get_parameter<double>("pose_tracking_pid_linear_y_i_clamp", pid_linear_y_params_.i_clamp);
  node_->get_parameter<double>("pose_tracking_pid_linear_z_p", pid_linear_z_params_.p);
  node_->get_parameter<double>("pose_tracking_pid_linear_z_i", pid_linear_z_params_.i);
  node_->get_parameter<double>("pose_tracking_pid_linear_zid", pid_linear_z_params_.d);
  node_->get_parameter<double>("pose_tracking_pid_linear_z_i_clamp", pid_linear_z_params_.i_clamp);
  node_->get_parameter<double>("pose_tracking_pid_angular_p", pid_angular_params_.p);
  node_->get_parameter<double>("pose_tracking_pid_angular_i", pid_angular_params_.i);
  node_->get_parameter<double>("pose_tracking_pid_angular_d", pid_angular_params_.d);
  node_->get_parameter<double>("pose_tracking_pid_angular_i_clamp", pid_angular_params_.i_clamp);

  //Convert angular i_clamp and angular goal tolerance to radians
  pid_angular_params_.i_clamp = toRadians(pid_angular_params_.i_clamp);
  goal_tolerance_angular_ = toRadians(goal_tolerance_angular_);

  //Initialise PID 
  std::size_t error = 0;
  error += !initPid(pid_linear_x_params_, pid_linear_x_, "pid_linear_x");
  error += !initPid(pid_linear_y_params_, pid_linear_y_, "pid_linear_y");
  error += !initPid(pid_linear_z_params_, pid_linear_z_, "pid_linear_z");
  error += !initPid(pid_angular_params_, pid_angular_, "pid_angular");

  RCLCPP_DEBUG(node_->get_logger(), "Initialized pose tracking 'tcp_frame':              %s", tcp_frame_.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "Initialized pose tracking 'goal_tolerance_linear':  %f", goal_tolerance_linear_);
  RCLCPP_DEBUG(node_->get_logger(), "Initialized pose tracking 'goal_tolerance_angular': %f", goal_tolerance_angular_);
  RCLCPP_DEBUG(node_->get_logger(), "Initialized pose tracking 'goal_settle_time':       %f", goal_settle_time_);
  RCLCPP_DEBUG(node_->get_logger(), "Initialized pose tracking 'receive_timeout':        %f", receive_timeout_);
}

bool PoseTrackingController::initPid(const motion_control_msgs::msg::PIDParameters &pid_params, control_toolbox::Pid& pid, const std::string &name)
{

  pid.initPid(pid_params.p, pid_params.i, pid_params.d, pid_params.i_clamp, -pid_params.i_clamp);

  RCLCPP_INFO(node_->get_logger(), "Initialized PID controller '%s'", name.c_str());

#ifndef NDEBUG
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.getGains(p, i, d, i_max, i_min, antiwindup);
  printf("  %s:\n"
         "    p:          %.3f\n"
         "    i:          %.3f\n"
         "    d:          %.3f\n"
         "    i_max:      %.3f\n"
         "    i_min:      %.3f\n"
         "    antiwindup: %d\n",
         name.c_str(), p, i, d, i_max, i_min, antiwindup);
#endif

  return true;
}

void PoseTrackingController::start()
{
  keep_running_ = true;
  control_task_ = std::thread(&PoseTrackingController::loop, this);
  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("pose_tracking/goal", 1, std::bind(&PoseTrackingController::poseGoalCb, this, std::placeholders::_1));
}

void PoseTrackingController::stop()
{
  if (keep_running_)
  {
    // sub_.shutdown();
    keep_running_ = false;
    goal_received_.notify_one();  // wake up thread if necessary
    if (control_task_.joinable())
      control_task_.join();
    reset();
  }
}

void PoseTrackingController::reset()
{
  pid_linear_x_.reset();
  pid_linear_y_.reset();
  pid_linear_z_.reset();
  pid_angular_.reset();
}

void PoseTrackingController::poseGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{

  rclcpp::Time timestamp = node_->now();
  std::lock_guard<std::mutex> lock(goal_mtx_);

  // check error state and recover after receive timeout
  if (in_error_ && (timestamp - goal_timestamp_).seconds() > receive_timeout_)
    in_error_ = false;

  goal_timestamp_ = timestamp;

  if (in_error_)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                             "Ignoring current pose goal stream when pose tracking was in error state. "
                             "Pose tracking will be available again after a receive timeout of %.2f seconds",
                             receive_timeout_);
  }
  else
  {
    // store pose goal and notify loop thread
    goal_ = *goal;
    goal_received_.notify_one();
  }
}

bool PoseTrackingController::hasReachedGoal(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                                            rclcpp::Time goal_timestamp)
{
  static rclcpp::Time goal_enter_timestamp;
  static bool goal_entered;

  double linear_error_abs = std::abs(linear_error.norm());
  double angular_error_abs = std::abs(angular_error.angle());

  // goal_enter_timestamp = std::max(goal_enter_timestamp, goal_timestamp);
  goal_enter_timestamp = goal_timestamp;
  // forcing the controller to stay active on new goals even if goal reached
  if (linear_error_abs <= goal_tolerance_linear_ && angular_error_abs <= goal_tolerance_angular_)
  {
    if (goal_entered)
    {
      if (node_->now() - goal_enter_timestamp >= rclcpp::Duration(goal_settle_time_, 0))
        return true;
    }
    else
    {
      goal_entered = true;
      goal_enter_timestamp = node_->now();
    }
  }
  else
  {
    goal_entered = false;
  }

  return false;
}

void PoseTrackingController::computeCommand(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                                            motion_control_msgs::msg::VelocityCommand& vel_cmd)
{
  auto& cmd = vel_cmd.cmd;

  printf("error: Vector[3] = [%.5f %.5f, %.5f]\n"
         "period: %s\n",
         linear_error.x(), linear_error.y(), linear_error.z(),
         std::to_string(controller_period_.nanoseconds()).c_str());

  cmd[0] = pid_linear_x_.computeCommand(linear_error.x(), controller_period_.nanoseconds());
  cmd[1] = pid_linear_y_.computeCommand(linear_error.y(), controller_period_.nanoseconds());
  cmd[2] = pid_linear_z_.computeCommand(linear_error.z(), controller_period_.nanoseconds());

  double angle_cmd = pid_angular_.computeCommand(angular_error.angle(), controller_period_.nanoseconds());
  cmd[3] = angle_cmd * angular_error.axis().x();
  cmd[4] = angle_cmd * angular_error.axis().y();
  cmd[5] = angle_cmd * angular_error.axis().z();

  // printf("Controller state:\n"
  //        "  command:  Vector6 = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
  //        "---\n",
  //        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
}

void PoseTrackingController::loop()
{
  RCLCPP_INFO(node_->get_logger(), "Ready to receive pose goals");

  while (rclcpp::ok() && keep_running_)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for new pose goal...");

    // sleep until new goal received
    {
      std::unique_lock<std::mutex> lock(goal_mtx_);
      reset();
      goal_received_.wait(lock);

      // check if everything is ok!
      if (!rclcpp::ok() || !keep_running_)
      {
        break;
      }
      // wait again, if it was a spurious wakeup
      if ((goal_timestamp_.nanoseconds() == last_goal_timestamp_.nanoseconds()) &&
          (goal_timestamp_.seconds() == last_goal_timestamp_.seconds()))
      {
        continue;
      }
      last_goal_timestamp_ = goal_timestamp_;

      // scoped lock gets unlocked so the goal can be updated in the callback

    }

    // try to start pose tracking mode and enter the control loop
    in_error_ = !driver_->startPoseTracking(motion_control_msgs::msg::VelocityCommand::BASE_FRAME);
    if (!in_error_)
    {

      //HIER VERDER CONTROLEREN
      in_error_ = !controlLoop();

      // set the driver in idle state
      driver_->stopMotionControl();
    }
  }
}

bool PoseTrackingController::controlLoop()
{

  // flag to exit control loop when the goal is reached within tolerance and settle time
  bool reached_goal = false;

  // goal data
  Eigen::Isometry3d goal_trsf;
  Eigen::Vector3d goal_position;
  Eigen::Quaterniond goal_orientation;
  rclcpp::Time goal_timestamp;
  std::string reference_frame;

  // feedback data
  Eigen::Isometry3d feedback_trsf;
  Eigen::Vector3d feedback_position;
  Eigen::Quaterniond feedback_orientation;
  rclcpp::Time last_feedback_time = node_->now();

  // velocity command
  motion_control_msgs::msg::VelocityCommand vel_cmd;
  vel_cmd.type = motion_control_msgs::msg::VelocityCommand::BASE_FRAME;

  RCLCPP_DEBUG(node_->get_logger(), "Entering control loop");
  while (rclcpp::ok() && keep_running_ && !reached_goal)
  {
    // store local copy of the goal data
    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      tf2::fromMsg(goal_.pose, goal_trsf);
      goal_timestamp = goal_timestamp_;
      reference_frame = goal_.header.frame_id;
    }

    // check if a reference frame is given or set default frame (base frame)
    const std::string& base_frame = driver_->getVelocityControlSettings().base_frame;
    const std::string& actual_reference_frame = reference_frame.empty() ? base_frame : reference_frame;
    if (reference_frame.empty())
    {
      RCLCPP_WARN_ONCE(node_->get_logger(), "No reference frame specified in goal pose. Using the base frame '%s' as reference frame by default", actual_reference_frame.c_str());
    }

    // transform goal into the base frame if necessary
    if (actual_reference_frame != base_frame)
    { 
      moveit::core::RobotState state = driver_->getRobotState();

      // check if the reference frame exists in the robot model
      if (!state.knowsFrameTransform(actual_reference_frame))
      {
        RCLCPP_ERROR(node_->get_logger(), "The reference frame '%s' of goal pose does not exist in the robot model",
                        actual_reference_frame.c_str());
        return false;
      }

      Eigen::Isometry3d reference_trsf =
          state.getGlobalLinkTransform(base_frame).inverse() * state.getGlobalLinkTransform(actual_reference_frame);

      // goal pose as is (w.r.t. to a reference frame)
      Eigen::Isometry3d goal_trsf_ref = goal_trsf;
      // goal pose transformed into the base frame (actually used)
      goal_trsf = reference_trsf * goal_trsf_ref;

#ifndef NDEBUG
      Eigen::Vector3d p = goal_trsf_ref.translation();
      Eigen::Quaterniond q = Eigen::Quaterniond(goal_trsf_ref.linear());
      printf("Goal (w.r.t. '%s'):\n"
             "  P.xyz  = [%.5f, %.5f, %.5f]\n"
             "  Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
             "---\n",
             actual_reference_frame.c_str(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());

      p = goal_trsf.translation();
      q = Eigen::Quaterniond(goal_trsf.linear());
      printf("Goal (w.r.t. '%s'):\n"
             "  P.xyz  = [%.5f, %.5f, %.5f]\n"
             "  Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
             "---\n",
             base_frame.c_str(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
#endif
    }

    // check if new goal arrived within timeout
    if (receive_timeout_ > 0 && (node_->now() - goal_timestamp_).seconds() >= receive_timeout_)
    {
      RCLCPP_DEBUG(node_->get_logger(), "No goal received until timeout. Exiting control loop.");
      break;
    }

    // get the most recent feedback from the robot (syncing with the robots interpolation clock)
    // TODO: decrease timeout for feedback and get from ROS parameter(?)
    if (!driver_->waitForFeedback(tcp_frame_, base_frame, last_feedback_time, 100, feedback_trsf))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get feedback from the robot. Exiting control loop.");
      return false;
    }

    // decompose transforms into translation and rotation
    goal_position = goal_trsf.translation();
    goal_orientation = Eigen::Quaterniond(goal_trsf.linear());
    feedback_position = feedback_trsf.translation();
    feedback_orientation = Eigen::Quaterniond(feedback_trsf.linear());

    // compute error
    Eigen::Vector3d linear_error = goal_position - feedback_position;
    Eigen::AngleAxisd angular_error = Eigen::AngleAxisd(goal_orientation * feedback_orientation.inverse());

    // check if goal reached
    if (hasReachedGoal(linear_error, angular_error, goal_timestamp))
    {
      // set flag to exit loop and send zero vector to stop the robot
      reached_goal = true;
      std::memset(vel_cmd.cmd.data(), 0, sizeof(vel_cmd.cmd));
    }
    else
    {
      computeCommand(linear_error, angular_error, vel_cmd);
    }

    // send command to robot
    if (!driver_->sendVelocityCommand(vel_cmd))
    {
      return false;
    }

#ifndef NDEBUG
    Eigen::Vector3d angular_error_vec = angular_error.axis() * angular_error.angle();
    const auto& cmd = vel_cmd.cmd;
    // clang-format off
    printf("Controller state:\n"
           "  goal:     Vector3 = [%.5f, %.5f, %.5f], Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
           "  feedback: Vector3 = [%.5f, %.5f, %.5f], Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
           "  error:    Vector6 = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
           "  command:  Vector6 = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
           "---\n",
           goal_position.x(), goal_position.y(), goal_position.z(),
           goal_orientation.x(), goal_orientation.y(), goal_orientation.z(), goal_orientation.w(),
           feedback_position.x(), feedback_position.y(), feedback_position.z(),
           feedback_orientation.x(), feedback_orientation.y(), feedback_orientation.z(), feedback_orientation.w(),
           linear_error.x(), linear_error.y(), linear_error.z(),
           angular_error_vec.x(), angular_error_vec.y(), angular_error_vec.z(),
           cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
    // clang-format on
#endif
  }

  return true;
}

}  // namespace robot_middleware
