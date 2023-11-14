/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#pragma once

#include "simple_message/byte_array.hpp"
#include "simple_message/shared_types.hpp"
#include "simple_message/simple_serialize.hpp"

namespace industrial
{
namespace velocity_config
{

class VelocityConfig : public industrial::simple_serialize::SimpleSerialize
{
public:
  VelocityConfig();

  ~VelocityConfig();

  void init(void);

  /**
   * Overrides - SimpleSerialize
   */
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + (4 + 2 * 6) * sizeof(industrial::shared_types::shared_real);
  }

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  industrial::shared_types::shared_int cmd_type_;
  industrial::shared_types::shared_real frame_ref_[6];
  industrial::shared_types::shared_real tool_ref_[6];
  industrial::shared_types::shared_real accel_;  // maximum joint acceleration in % of the nominal acceleration [0,1]
  industrial::shared_types::shared_real vel_;    // maximum joint velocity in % of the nominal velocity [0,1]
  industrial::shared_types::shared_real tvel_;   // maximum linear velocity of the tool-center-point in m/s
  industrial::shared_types::shared_real rvel_;   // maximum angular velocity of the tool-center-point in rad/s
};

}  // namespace industrial
}  // namespace velocity_config
