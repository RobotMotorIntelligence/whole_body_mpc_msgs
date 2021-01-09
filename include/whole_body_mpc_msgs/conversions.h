//
// Copyright (c) 2021, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_
#define WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_

#include <Eigen/Core>
#include <whole_body_mpc_msgs/StateFeedbackGain.h>

namespace whole_body_mpc_msgs {
/**
 * @brief Conversion of Eigen::MatrixXd to message for a given whole_body_mpc_msgs::StateFeedbackGain message reference
 *
 * @param msg whole_body_mpc_msgs::StateFeedbackGain (reference, will be modified)
 * @param K state feedback gain (size nu * nx)
 */
void toMsg(whole_body_mpc_msgs::StateFeedbackGain &msg, const Eigen::MatrixXd &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (auto i = 0; i < msg.nu; ++i)
    for (auto j = 0; j < msg.nx; ++j) msg.data[i * msg.nx + j] = K(i, j);
}

/**
 * @brief Conversion of a state feedback gain from a whole_body_mpc_msgs::StateFeedbackGain message to an
 * Eigen::MatrixXd
 *
 * @param msg whole_body_mpc_msgs::StateFeedbackGain
 * @param K state feedback gain (size nu * nx) (reference, will be modified)
 */
void fromMsg(const whole_body_mpc_msgs::StateFeedbackGain &msg, Eigen::Ref<Eigen::MatrixXd> K) {
  if (K.rows() != msg.nu || K.cols() != msg.nx) {
    K.resize(msg.nu, msg.nx);
  }
  if (msg.data.size() != msg.nu * msg.nx) {
    throw std::invalid_argument("Message incorrect - size of data does not match given dimensions (nu,nx)");
  }

  for (std::size_t i = 0; i < msg.nu; ++i)
    for (std::size_t j = 0; j < msg.nx; ++j) K(i, j) = msg.data[i * msg.nx + j];
}
}  // namespace whole_body_mpc_msgs

#endif  // WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_
