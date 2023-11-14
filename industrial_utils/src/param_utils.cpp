
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include <sstream>
#include <chrono>

using namespace std::chrono_literals;

#include "industrial_utils/param_utils.hpp"
#include "industrial_utils/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf_extention/model_extention.hpp"

namespace industrial_utils
{
namespace param
{

ParamUtils::ParamUtils() : Node("param_utils")
{
}
bool ParamUtils::getListParam(const std::string thisname, const std::string param_name, std::vector<std::string> & list_param)
{
  bool rtn = false;
  std::vector<rclcpp::Parameter> rpc_lists = {};
  rclcpp::Parameter rpc_list;
  list_param.clear();
  
  //Prepare the parameter sync to obtain parameters from other nodes
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, thisname);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  if(parameters_client->has_parameter(param_name)) 
  {
    rtn = true;
  }
  rpc_lists = parameters_client->get_parameters({param_name});
  rpc_list = rpc_lists[0];

  if(rtn)
  {
    rtn = (rpc_list.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    if(rtn)
    {
      for (int i = 0; i < rpc_list.as_string_array().size(); ++i)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Adding " << rpc_list.as_string_array().at(i) << " to list parameter");
        list_param.push_back(static_cast<std::string>(rpc_list.as_string_array().at(i)));
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter: " + param_name + " of node: " + thisname + " is not of type string_array");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed ot get parameter: " + param_name);
  }

  return rtn;
}

std::string ParamUtils::vec2str(const std::vector<std::string> &vec)
{
  std::string s, delim = ", ";
  std::stringstream ss;
  std::copy(vec.begin(), vec.end(), std::ostream_iterator<std::string>(ss, delim.c_str()));
  s = ss.str();
  return "[" + s.erase(s.length()-2) + "]";
}

bool ParamUtils::getJointNames(const std::string joint_names_thisname, const std::string urdf_thisname, const std::string joint_list_param, const std::string urdf_param,
		           std::vector<std::string> & joint_names)
{
  joint_names.clear();

  // 1) Try to read explicit list of joint names
  // Setup service client to get the params out of the desired node
  auto joint_names_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, joint_names_thisname);
  while (!joint_names_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  
  if (joint_names_parameters_client->has_parameter(joint_list_param) && getListParam(joint_names_thisname, joint_list_param, joint_names))
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Found user-specified joint names in '" << joint_list_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to find user-specified joint names in '" << joint_list_param << "'" << " from node: '" << joint_names_thisname << "'");

  // 2) Try to find joint names from URDF model
  // Setup another service client for urdf param
  auto urdf_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, urdf_thisname);
  while (!urdf_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  urdf::ModelExtention model;
  if ( urdf_parameters_client->has_parameter(urdf_param)
       && model.initParam(urdf_thisname, urdf_param)
       && findChainJointNames(model.getRoot(), true, joint_names) )
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Using joint names from URDF: '" << urdf_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to find URDF joint names in '" << urdf_param << "'" << " at node: " << urdf_thisname << "'");

  // 3) Raise an error
  RCLCPP_ERROR_STREAM(this->get_logger(),
      "Cannot find user-specified joint names. Tried ROS parameter '" << joint_list_param << "'"
      << " and the URDF in '" << urdf_param << "'.");
  return false;
}
bool ParamUtils::getJointNames(const std::string joint_names_thisname, const std::string joint_list_param,
		           std::vector<std::string> & joint_names)
{
  joint_names.clear();

  // 1) Try to read explicit list of joint names
  // Setup service client to get the params out of the desired node
  auto joint_names_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, joint_names_thisname);
  while (!joint_names_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  
  if (joint_names_parameters_client->has_parameter(joint_list_param) && getListParam(joint_names_thisname, joint_list_param, joint_names))
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Found joint names from node '" << joint_names_thisname << " in '" << joint_list_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to find joint names in '" << joint_list_param << "'" << " from node: '" << joint_names_thisname << "'");
  return false;
}

bool ParamUtils::getJointVelocityLimits(const std::string thisname, const std::string urdf_param_name, std::map<std::string, double> &velocity_limits)
{
  // Setup service client to get the params out of the desired node
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, thisname);

  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  urdf::ModelExtention model;
  std::map<std::string, urdf::JointSharedPtr >::iterator iter;

  if (!parameters_client->has_parameter(urdf_param_name) || !model.initParam(thisname, urdf_param_name))
    return false;
    
  velocity_limits.clear();
  for (iter=model.joints_.begin(); iter!=model.joints_.end(); ++iter)
  {
    std::string joint_name(iter->first);
    urdf::JointLimitsSharedPtr limits = iter->second->limits;
    if ( limits && (limits->velocity > 0) )
      velocity_limits.insert(std::pair<std::string,double>(joint_name,limits->velocity));
  }
  
  return true;
}

} //industrial_utils::param
} //industrial_utils
