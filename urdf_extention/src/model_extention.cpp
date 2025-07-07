/*
 * Copyright 2025 ACRO - KULeuven
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

#include "urdf_extention/model_extention.hpp"
using namespace std::chrono_literals;

namespace urdf
{
ModelExtention::ModelExtention() : Model(), Node("model_extention")
{
}
ModelExtention::~ModelExtention()
{
}
bool ModelExtention::initParam(const std::string &node_name, const std::string &param_name)
{
    std::vector <rclcpp::Parameter> xml_strings = {};
    rclcpp::Parameter xml_string;

    rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

    if(!parameters_client->has_parameter(param_name))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter %s not found in node %s", param_name.c_str(), node_name.c_str());
        return false;
    }

    xml_strings = parameters_client->get_parameters({param_name});
    xml_string = xml_strings[0];

    if(xml_string.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter %s of node %s is not of type string", param_name.c_str(), node_name.c_str());
    }
    
    // RCLCPP_INFO(this->get_logger(), xml_string.as_string());

    return this->initString(xml_string.as_string());
}
}
