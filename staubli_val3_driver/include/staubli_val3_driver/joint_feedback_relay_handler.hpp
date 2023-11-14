
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

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "simple_message/message_handler.hpp"
#include "simple_message/messages/joint_feedback_message.hpp"

#include "rclcpp/rclcpp.hpp"


/**
 * @brief Message handler that relays joint positions
 * (Converts simple message types to ROS message types and publishes them)
 * 
 */
class JointFeedbackRelayHandler : public industrial::message_handler::MessageHandler, public rclcpp::Node
{
  using industrial::message_handler::MessageHandler::init;

public:
  /**
   * @brief Construct a new Joint Feedback Relay Handler object
   * 
   */
  JointFeedbackRelayHandler();

  /**
   * @brief Class initialiser
   * 
   * @param connection simple message connection that will be used to send replies
   * @param joint_names list of joint-names for msg-publishing.
   *    - Count and order should match data from robot connection.
   *    - Use blank-name to exclude a joint from publishing
   * @return true on success
   * @return false otherwise
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection *connection, 
            std::vector<std::string> &joint_names);

protected:

  /**
   * @brief Convert joint feedback message into publish message-types
   * 
   * @param[in] msg_in JointFeedback message from robot connection
   * @param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * @param[out] sensor_state ointState message for ROS publishing
   * @return true on success
   * @return false otherwise
   */
  virtual bool createMessages(industrial::joint_feedback_message::JointFeedbackMessage &msg_in,
                              control_msgs::action::FollowJointTrajectory_FeedbackMessage *control_state,
                              sensor_msgs::msg::JointState *sensor_state);

  /**
   * @brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   * 
   * @param[in] pos_in joint positions, exactly as passed from robot connection.
   * @param[out] pos_out transformed joint positions (in same order/count as input positions)
   *
   * @return true on success
   * @return false otherwise
   */
  virtual bool transform(const std::vector<double> &pos_in,
                         std::vector<double> *pos_out);

  /**
   * @brief Select specific joints for publishing
   * 
   * @param[in] all_joint_pos joint positions, in count/order matching robot connection
   * @param[in] all_joint_names joint names, matching all_joint_pos
   * @param[out] pub_joint_pos joint positions selected for publishing
   * @param[out] pub_joint_names joint names selected for publishing
   * @return true on success
   * @return false otherwise
   */
  virtual bool select(const std::vector<double> &all_joint_pos,
                      const std::vector<std::string> &all_joint_names,
                      std::vector<double> *pub_joint_pos,
                      std::vector<std::string> *pub_joint_names);

  /**
   * @brief Callback executed upon receiving a joint feedback message
   * 
   * @param in incoming message
   * @return true on success
   * @return false otherwise
   */
  bool internalCB(industrial::joint_feedback_message::JointFeedbackMessage &in);


  std::vector<std::string> all_joint_names_;
  rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_FeedbackMessage>::SharedPtr pub_joint_control_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_sensor_state_;

private:
  /**
   * @brief Callback executed upon receiving a message
   * 
   * @param in incoming message
   * @return true on success
   * @return false otherwise
   */
  bool internalCB(industrial::simple_message::SimpleMessage &in);

};