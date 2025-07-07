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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "industrial_msgs/msg/robot_status.hpp"
#include "industrial_robot_client/utils.hpp"
#include "industrial_utils/param_utils.hpp"
#include "industrial_utils/utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include "industrial_robot_client/visibility_control.h"


namespace industrial_robot_client
{
namespace joint_trajectory_action
{

class JointTrajectoryAction : public rclcpp::Node
{
public:
    using FJT = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

    INDUSTRIAL_ROBOT_CLIENT_PUBLIC

    /**
     * \brief Constructor
     *
     */
    explicit JointTrajectoryAction(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("joint_trajectory_action", options)
    {
        using namespace std::placeholders;
        joint_names_ = {};

        has_active_goal_ = false;
        controller_alive_ = false;
        has_moved_once_ = false;
        has_succeeded_ = false;
        name_ = "manipulator_controller/joint_trajectory_action";
        action_server_ = rclcpp_action::create_server<FJT>(
            this,
            "manipulator_controller/joint_trajectory_action",
            std::bind(&JointTrajectoryAction::handle_goal, this, _1, _2),
            std::bind(&JointTrajectoryAction::handle_cancel, this, _1),
            std::bind(&JointTrajectoryAction::handle_accepted, this, _1));

        this->declare_parameter<double>("constraints/goal_threshold", DEFAULT_GOAL_THRESHOLD_);
        this->get_parameter<double>("constraints/goal_threshold", goal_threshold_);

        industrial_utils::param::ParamUtils pu;
        // if (!pu.getJointNames("move_group", "rviz2", "controller_joint_names", "robot_description", joint_names_))
        if (!pu.getJointNames("move_group", "moveit_simple_controller_manager.manipulator_controller.joints", joint_names_))

            RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint_names.");

        // The controller joint names parameter includes empty joint names for those joints not supported
        // by the controller.  These are removed since the trajectory action should ignore these.
        std::remove(joint_names_.begin(), joint_names_.end(), std::string());
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtered joint names to " << joint_names_.size() << " joints");

        pub_trajectory_command_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 1);
        sub_trajectory_state_ = this->create_subscription<control_msgs::action::FollowJointTrajectory_FeedbackMessage>("feedback_states", 1, std::bind(&JointTrajectoryAction::controllerStateCB, this, _1));
        sub_robot_status_ = this->create_subscription<industrial_msgs::msg::RobotStatus>("robot_status", 1, std::bind(&JointTrajectoryAction::robotStatusCB, this, _1));

        auto clock_ = rclcpp::Clock::make_shared();
        watchdog_timer_ = rclcpp::create_timer(this, clock_, std::chrono::seconds(WATCHDOG_PERIOD_), std::bind(&JointTrajectoryAction::watchdog, this));
    }


    /**
     * \brief Destructor
     *
     */
    ~JointTrajectoryAction()
    {
    }

    /**
      * \brief Begin processing messages and publishing topics.
    */
    void run() 
    {
        rclcpp::spin(this->shared_from_this());
    }

private:
    /**
     * \brief Name of this class, for logging namespacing
     */
    std::string name_;

    /**
     * \brief Internal action server
     */
    rclcpp_action::Server<FJT>::SharedPtr action_server_;

    /**
     * \brief Publishes desired trajectory (typically to the robot driver)
     */
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_command_;

    /**
     * \brief Subscribes to trajectory feedback (typically published by the
     * robot driver).
     */
    rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_FeedbackMessage>::SharedPtr sub_trajectory_state_;

    /**
     * \brief Subscribes to the robot status (typically published by the
     * robot driver).
     */
    rclcpp::Subscription<industrial_msgs::msg::RobotStatus>::SharedPtr sub_robot_status_;

    /**
     * \brief Watchdog time used to fail the action request if the robot
     * driver is not responding.
     */
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    /**
     * \brief Controller was alive during the last watchdog interval
     */
    bool controller_alive_;

    /**
     * \brief Indicates action has an active goal
     */
    bool has_active_goal_;

    /**
     * \brief Indicates that the robot has been in a moving state at least once since
     * starting the current active trajectory
     */
    bool has_moved_once_;

    /**
     * \brief Indicates if the robot has successfully finished the trajectory 
     * 
     */
    bool has_succeeded_;

    /**
     * \brief Cache of the current active goal
     */
    FJT::Goal active_goal_;

    /**
     * \brief Cache of the current active goal
     */
    std::shared_ptr<GoalHandleFJT> goal_handle_;

    /**
     * \brief Cache of the current active trajectory
     */
    trajectory_msgs::msg::JointTrajectory current_traj_;

    /**
     * \brief The default goal joint threshold see(goal_threshold). Unit
     * are joint specific (i.e. radians or meters).
     */
    const double DEFAULT_GOAL_THRESHOLD_ = 0.01; // = 0.01;
    // const double default_goal_threshold_ = 0.01; // = 0.01;

    /**
     * \brief The goal joint threshold used for determining if a robot
     * is near it final destination.  A single value is used for all joints
     *
     * NOTE: This value is used in conjunction with the robot inMotion
     * status (see industrial_msgs::RobotStatus) if it exists.
     */
    double goal_threshold_;

    /**
     * \brief The joint names associated with the robot the action is
     * interfacing with.  The joint names must be the same as expected
     * by the robot driver.
     */
    std::vector<std::string> joint_names_;

    /**
     * \brief Cache of the last subscribed feedback message
     */
    control_msgs::action::FollowJointTrajectory_FeedbackMessage::SharedPtr last_trajectory_state_;

    /**
     * \brief Cache of the last subscribed status message
     */
    industrial_msgs::msg::RobotStatus::SharedPtr last_robot_status_;

    /**
     * \brief Time at which to start checking for completion of current
     * goal, if one is active
     */
    rclcpp::Time time_to_check_;

    /**
     * \brief The watchdog period (seconds)
     */
    const int WATCHDOG_PERIOD_ = 1.0; // = 1.0;
    // const int watchdog_period_ = 1.0; // = 1.0;

    /**
     * \brief Watch dog callback, used to detect robot driver failures
     *
     *
     */
    void watchdog()
    {
        // Some debug logging
        if (!last_trajectory_state_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for subscription to joint trajectory state");
        }

        RCLCPP_WARN(this->get_logger(), "Trajectory state not received for %i seconds", WATCHDOG_PERIOD_);
        controller_alive_ = false;
        has_succeeded_ = false;


        // Aborts the active goal if the controller does not appear to be active.
        if (has_active_goal_)
        {
            // last_trajectory_state_ is null if the subscriber never makes a connection
            if (!last_trajectory_state_)
            {
                RCLCPP_WARN(this->get_logger(), "Aborting goal because we have never heard a controller state message.");
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
            }

            abortGoal();
        }
    }

    /**
     * \brief Action server goal callback method
     *
     */

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                        std::shared_ptr<const FJT::Goal> goal)
    {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received new goal");  
            has_succeeded_ = false;

            //If the controller is nog alive, we reject the incoming goal request
            if(!controller_alive_)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory action rejected: waiting for (initial) feedback from controller");
                (void)uuid;
                return rclcpp_action::GoalResponse::REJECT;
            }
            if(!goal->trajectory.points.empty())
            {
                if(industrial_utils::isSimilar(joint_names_, goal->trajectory.joint_names))
                {
                    if(has_active_goal_)
                    {
                        RCLCPP_WARN(this->get_logger(), "Received new goal, canceling current goal");
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    active_goal_ = *goal;
            
                    builtin_interfaces::msg::Duration temp = active_goal_.trajectory.points.back().time_from_start;
                    // RCLCPP_INFO(this->get_logger(), "sec: %i", test.seconds());
                    // RCLCPP_INFO(this->get_logger(), "nsec: %i", test.nanoseconds());
                    temp.sec = temp.sec/2;
                    temp.nanosec = temp.nanosec/2;
                    rclcpp::Duration test(temp);
                    time_to_check_ = this->now() + rclcpp::Duration(temp);
                    has_moved_once_ = false;
                    current_traj_ = active_goal_.trajectory;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Joint trajectory action failing on invalid joints");
                    (void)uuid;
                    return rclcpp_action::GoalResponse::REJECT;
                } 
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory action failed on empty trajectory");
                (void)uuid;
                return rclcpp_action::GoalResponse::REJECT;
            }
            //Add some informational log messages to indicate supported goal constraints
            if(goal->goal_time_tolerance.sec > 0.0)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring goal time tolerance in action goal, may be supported in the future");
            }
            if(!goal->goal_tolerance.empty())
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring goal tolerance in action, using paramater tolerance of " + std::to_string(goal_threshold_) + " instead");
            }
            if(!goal->path_tolerance.empty())
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
            }
            (void)uuid;
            has_active_goal_ = true;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * \brief Action server cancel callback method
     *
     * \param gh goal handle
     *
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received action cancel request");
        if(goal_handle_ == goal_handle)
        {
            has_active_goal_ = false;
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Active goal and goal cancel do not match, ignoring cancel request");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::REJECT;
        }
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        std::thread{std::bind(&JointTrajectoryAction::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        goal_handle_ = goal_handle;
        const auto goal = goal_handle->get_goal();
        std::shared_ptr<FJT::Feedback> feedback = std::make_shared<FJT::Feedback>();
        std::shared_ptr<FJT::Result> result  = std::make_shared<FJT::Result>();
        
        pub_trajectory_command_->publish(current_traj_);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback");

        while (rclcpp::ok())
        {
            if(goal_handle->is_canceling())
            {
                trajectory_msgs::msg::JointTrajectory empty;
                empty.joint_names = joint_names_;
                pub_trajectory_command_->publish(empty);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                has_succeeded_ = false;
                return;
            }
            if(has_succeeded_)
            {
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                has_succeeded_ = false;
                has_active_goal_ = false;
                break;
            }
        }
        
    }
    /**
     * \brief Controller state callback (executed when feedback message
     * received)
     *
     * \param msg joint trajectory feedback message
     *
     */

    void controllerStateCB(const control_msgs::action::FollowJointTrajectory_FeedbackMessage::SharedPtr msg)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Checking controller state feedback");

        last_trajectory_state_ = msg;
        controller_alive_ = true;

        watchdog_timer_->reset();

        if (!has_active_goal_)
        {
            //ROS_DEBUG_NAMED(name_, "No active goal, ignoring feedback");
            return;
        }
        if (current_traj_.points.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Current trajectory is empty, ignoring feedback");
            return;
        }

        if (!industrial_utils::isSimilar(joint_names_, msg->feedback.joint_names))
        {
            RCLCPP_ERROR(this->get_logger(), "Joint names from the controller don't match our joint names.");
            return;
        }

        if (!has_moved_once_ && (this->now() < time_to_check_))
        {
            // RCLCPP_INFO(this->get_logger(), "Waiting to check for goal completion until halfway through trajectory");
            return;
        }

        // Checking for goal constraints
        // Checks that we have ended inside the goal constraints and has motion stopped

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Checking goal constraints");
        if (withinGoalConstraints(last_trajectory_state_, current_traj_))
        {
            if (last_robot_status_)
            {
            // Additional check for motion stoppage since the controller goal may still
            // be moving.  The current robot driver calls a motion stop if it receives
            // a new trajectory while it is still moving.  If the driver is not publishing
            // the motion state (i.e. old driver), this will still work, but it warns you.
            if (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::FALSE && last_robot_status_->trajectory_complete.val == industrial_msgs::msg::TriState::TRUE)
            {
                RCLCPP_INFO(this->get_logger(), "Inside goal constraints - stopped moving-  return success for action");
                has_active_goal_ = false;
                has_succeeded_ = true;
            }
            else if (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::UNKNOWN)
            {
                RCLCPP_INFO(this->get_logger(), "Inside goal constraints, return success for action");
                RCLCPP_WARN(this->get_logger(), "Robot status in motion unknown, the robot driver node and controller code should be updated");
                has_active_goal_ = false;
                has_succeeded_ = true;
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Within goal constraints but robot is still moving");
            }
            }
            else
            {
            RCLCPP_INFO(this->get_logger(), "Inside goal constraints, return success for action");
            RCLCPP_WARN(this->get_logger(), "Robot status is not being published the robot driver node and controller code should be updated");
            has_active_goal_ = false;
            has_succeeded_ = true;
            }
        }
    }

    /**
     * \brief Controller status callback (executed when robot status
     *  message received)
     *
     * \param msg robot status message
     *
     */
    void robotStatusCB(const industrial_msgs::msg::RobotStatus::SharedPtr msg)
    {
        last_robot_status_= msg; //caching robot status for later use.
        has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::TRUE);
    }

    /**
     * \brief Aborts the current action goal and sends a stop command
     * (empty message) to the robot driver.
     *
     *
     */
    void abortGoal()
    {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        pub_trajectory_command_->publish(empty);
        std::shared_ptr<FJT::Result> result = std::make_shared<FJT::Result>();

        goal_handle_->abort(result);

        // Marks the current goal as aborted.
        has_active_goal_ = false;
    }

    /**
     * \brief Controller status callback (executed when robot status
     *  message received)
     *
     * \param msg trajectory feedback message
     * \param traj trajectory to test against feedback
     *
     * \return true if all joints are within goal contraints
     *
     */
    bool withinGoalConstraints(const control_msgs::action::FollowJointTrajectory_FeedbackMessage::SharedPtr &msg,
                               const trajectory_msgs::msg::JointTrajectory &traj)
    {
        (void)*msg;
        if (traj.points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty joint trajectory passed to check goal constraints, return false");
            return false;
        }
        else
        {
            int last_point = traj.points.size() - 1;
            if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->feedback.joint_names,
                                                              last_trajectory_state_->feedback.actual.positions, 
                                                              traj.joint_names,
                                                              traj.points[last_point].positions, 
                                                              goal_threshold_))
            {
            return true;
            }
            else
            {
            return false;
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    return 0;
}


} // joint_trajectory_action
} // industrial_robot_client
RCLCPP_COMPONENTS_REGISTER_NODE(industrial_robot_client::joint_trajectory_action::JointTrajectoryAction)