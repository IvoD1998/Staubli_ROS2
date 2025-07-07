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

// Modified from original version in https://github.com/FAU-FAPS/adaptive_motion_control
// Changes made to support CS9 and ROS 2 compatibility.
// Copyright 2025 ACRO - KULeuven

#pragma once

#include <Eigen/Dense>

namespace eigen_typedefs
{

const unsigned int MAX_DIMENSION = 10;

using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_DIMENSION, MAX_DIMENSION>;
using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MAX_DIMENSION, 1>;

}  // namespace eigen_typedefs