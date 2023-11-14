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

#include "industrial_robot_client/robot_status_relay_handler.hpp"

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

namespace industrial_robot_client
{
namespace robot_status_relay_handler
{

RobotStatusRelayHandler::RobotStatusRelayHandler() : Node("robot_status_relay_handler")
{
}

bool RobotStatusRelayHandler::init(industrial::smpl_msg_connection::SmplMsgConnection *connection)
{
  pub_robot_status_ = this->create_publisher<industrial_msgs::msg::RobotStatus>("robot_status", 1);
  return init((int)industrial::simple_message::StandardMsgTypes::STATUS, connection);
}

bool RobotStatusRelayHandler::internalCB(industrial::simple_message::SimpleMessage &in)
{
  industrial::robot_status_message::RobotStatusMessage status_msg;

  if (!status_msg.init(in))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize status message");
    return false;
  }

  return internalCB(status_msg);
}

bool RobotStatusRelayHandler::internalCB(industrial::robot_status_message::RobotStatusMessage & in)
{
  industrial_msgs::msg::RobotStatus status;
  bool rtn = true;

  status.drives_powered.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getDrivesPowered());
  status.e_stopped.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getEStopped());
  status.error_code = in.status_.getErrorCode();
  status.in_error.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInError());
  status.in_motion.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInMotion());
  status.mode.val = industrial::robot_status::RobotModes::toROSMsgEnum(in.status_.getMode());
  status.motion_possible.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getMotionPossible());
  
  pub_robot_status_->publish(status);

  // Reply back to the controller if the sender requested it.
  if (industrial::simple_message::CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    industrial::simple_message::SimpleMessage reply;
    in.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS : industrial::simple_message::ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

}
}