
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
#include "rcpputils/asserts.hpp"

JointFeedbackRelayHandler::JointFeedbackRelayHandler() : Node("joint_feedback_relay_handler")
{
}

bool JointFeedbackRelayHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection, 
                                     std::vector<std::string> &joint_names)
{
    pub_joint_control_state_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_FeedbackMessage>("feedback_states", 1);
    pub_joint_sensor_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    all_joint_names_ = joint_names;

    return init(static_cast<int>(industrial::simple_message::StandardMsgType::JOINT_FEEDBACK), connection);
}

bool JointFeedbackRelayHandler::createMessages(industrial::joint_feedback_message::JointFeedbackMessage &msg_in,
                                               control_msgs::action::FollowJointTrajectory_FeedbackMessage *control_state,
                                               sensor_msgs::msg::JointState *sensor_state)
{
    //Read all joint positions/velocities from JointFeedbackMessage
    std::vector<double> all_joint_pos(all_joint_names_.size());
    std::vector<double> all_joint_vel(all_joint_names_.size());

    industrial::joint_data::JointData pos;
    industrial::joint_data::JointData vel;

    //Check if valid
    bool has_pos = msg_in.getPositions(pos);
    bool has_vel = msg_in.getVelocities(vel);

    if(has_pos)
    {
        for (int i = 0; i < all_joint_names_.size(); ++i)
        {
            industrial::shared_types::shared_real value;
            if(pos.getJoint(i, value)) all_joint_pos[i] = value;
            else RCLCPP_ERROR(this->get_logger(), "Failed to parse position value #%d from JointFeedbackMessage", i);
        }
    }
    if(has_vel)
    {
        for (int i = 0; i < all_joint_names_.size(); ++i)
        {
            industrial::shared_types::shared_real value;
            if(vel.getJoint(i, value)) all_joint_vel[i] = value;
            else RCLCPP_ERROR(this->get_logger(), "Failed to parse velocity value #%d from JointFeedbackMessage", i);
        }
    }

    //Select specific joints for publishing
    std::vector<double> pub_joint_pos;
    std::vector<double> pub_joint_vel;
    std::vector<std::string> pub_joint_names;

    if (has_pos)
    {
        if(!select(all_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to select joint positions for publishing");
            return false;
        }
    }

    if (has_vel)
    {
        if(!select(all_joint_vel, all_joint_names_, &pub_joint_vel, &pub_joint_names))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to select joint velocities for publishing");
            return false;
        }
    }

    //Assign values to messages
    control_msgs::action::FollowJointTrajectory_FeedbackMessage tmp_control_state;
    tmp_control_state.feedback.header.stamp = this->now();
    tmp_control_state.feedback.joint_names = pub_joint_names;
    if(has_pos) tmp_control_state.feedback.actual.positions = pub_joint_pos;
    if(has_vel) tmp_control_state.feedback.actual.velocities = pub_joint_vel;
    *control_state = tmp_control_state;

    sensor_msgs::msg::JointState tmp_sensor_state;
    tmp_sensor_state.header.stamp = this->now();
    tmp_sensor_state.name = pub_joint_names;
    if(has_pos) tmp_sensor_state.position = pub_joint_pos;
    if(has_vel) tmp_sensor_state.velocity = pub_joint_vel;
    *sensor_state = tmp_sensor_state;

    return true;
}

bool JointFeedbackRelayHandler::select(const std::vector<double> &all_joint_pos, 
                                      const std::vector<std::string> &all_joint_names,
                                      std::vector<double> *pub_joint_pos, 
                                      std::vector<std::string> *pub_joint_names)
{
    rcpputils::assert_true(all_joint_pos.size() == all_joint_names.size());
    pub_joint_pos->clear();
    pub_joint_names->clear();

    //Skip over "blank" joint names
    for (int i = 0; i < all_joint_pos.size(); ++i)
    {
        if (all_joint_names[i].empty()) continue;
        pub_joint_pos->push_back(all_joint_pos[i]);
        pub_joint_names->push_back(all_joint_names[i]);
    }
    return true;
}

bool JointFeedbackRelayHandler::transform(const std::vector<double> &pos_in, 
                                          std::vector<double> *pos_out)
{
    *pos_out = pos_in;
    return true;
}

bool JointFeedbackRelayHandler::internalCB(industrial::simple_message::SimpleMessage &in)
{  
    industrial::joint_feedback_message::JointFeedbackMessage joint_fbk_msg;
    if(!joint_fbk_msg.init(in))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint feedback message");
        return false;
    }
    return internalCB(joint_fbk_msg);
}

bool JointFeedbackRelayHandler::internalCB(industrial::joint_feedback_message::JointFeedbackMessage &in)
{
    control_msgs::action::FollowJointTrajectory_FeedbackMessage control_state;
    sensor_msgs::msg::JointState sensor_state;
    bool rtn = true;

    if(createMessages(in, &control_state, &sensor_state))
    {
        pub_joint_control_state_->publish(control_state);
        pub_joint_sensor_state_->publish(sensor_state);
    }
    else rtn = false;

    //Reply back to the controller if the sender requested it
    if (industrial::simple_message::CommTypes::SERVICE_REQUEST == in.getMessageType())
    {
        industrial::simple_message::SimpleMessage reply;
        in.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS : industrial::simple_message::ReplyTypes::FAILURE);
        this->getConnection()->sendMsg(reply);
    }
    return rtn;
}


