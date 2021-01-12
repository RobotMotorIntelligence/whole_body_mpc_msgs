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

#ifndef WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_
#define WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_

#include <whole_body_mpc_msgs/conversions.h>

namespace whole_body_mpc_msgs {
class StateFeedbackGainInterface {
 public:
  StateFeedbackGainInterface(const std::size_t nx, const std::size_t nu, const std::string &frame_id = "odom")
      : nx_(nx), nu_(nu), K_(Eigen::MatrixXd(nu, nx)) {
    // Setup message
    msg_.header.frame_id = frame_id;
    msg_.nx = nx_;
    msg_.nu = nu_;
    msg_.data.resize(nx_ * nu_);
  }

  /**
   * @brief Conversion of Eigen::MatrixXd to message, returns a whole_body_mpc_msgs::StateFeedbackGain message
   *
   * @param K state feedback gain (size nu * nx)
   * @return whole_body_mpc_msgs::StateFeedbackGain
   */
  whole_body_mpc_msgs::StateFeedbackGain writeToMessage(const Eigen::MatrixXd &K) {
    if (K.rows() != nu_ || K.cols() != nx_) {
      throw std::invalid_argument("Expected K to be (" + std::to_string(nu_) + ", " + std::to_string(nx_) +
                                  ") but received (" + std::to_string(K.rows()) + ", " + std::to_string(K.cols()) +
                                  ").");
    }

    toMsg(msg_, K);
    return msg_;
  }

  /**
   * @brief Conversion of whole_body_mpc_msgs::StateFeedbackGain message to Eigen::MatrixXd to message, returns a
   * Eigen::MatrixXd
   *
   * @param msg whole_body_mpc_msgs::StateFeedbackGain
   * @return const Eigen::MatrixXd&
   */
  const Eigen::MatrixXd &writeFromMessage(const whole_body_mpc_msgs::StateFeedbackGain &msg) {
    if (msg.data.size() != nu_ * nx_ || msg.nu != nu_ || msg.nx != nx_) {
      throw std::invalid_argument("Dimensions of message do not match nx, nu of StateFeedbackGainInterface.");
    }
    fromMsg(msg, K_);
    return K_;
  }

 protected:
  std::size_t nx_;
  std::size_t nu_;

  whole_body_mpc_msgs::StateFeedbackGain msg_;
  Eigen::MatrixXd K_;
};
}  // namespace whole_body_mpc_msgs

#endif  // WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_
