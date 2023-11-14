#pragma once
#include "urdf/model.h"
#include "rclcpp/rclcpp.hpp"

namespace urdf
{
class ModelExtention : public urdf::Model, public rclcpp::Node
{
public:
    URDF_EXPORT
    ModelExtention();

    URDF_EXPORT
    ~ModelExtention();

    URDF_EXPORT bool initParam(const std::string &node, const std::string &param);

};
} //namespace urdf
