///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_
#define WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_

#include <Eigen/Core>
#include <whole_body_mpc_msgs/StateFeedbackGain.h>

namespace whole_body_mpc_msgs {
/**
 * @brief Conversion of Eigen::MatrixXd to message for a given whole_body_mpc_msgs::StateFeedbackGain message reference
 *
 * @param[out] msg whole_body_mpc_msgs::StateFeedbackGain (reference, will be modified)
 * @param[in] K state feedback gain (size nu * nx)
 */
static inline void toMsg(whole_body_mpc_msgs::StateFeedbackGain &msg, const Eigen::MatrixXd &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (uint32_t i = 0; i < msg.nu; ++i)
    for (uint32_t j = 0; j < msg.nx; ++j) msg.data[i * msg.nx + j] = K(i, j);
}

/**
 * @brief Conversion of a state feedback gain from a whole_body_mpc_msgs::StateFeedbackGain message to an
 * Eigen::MatrixXd
 *
 * @param[in] msg whole_body_mpc_msgs::StateFeedbackGain
 * @param[out] K state feedback gain (size nu * nx) (reference, will be modified)
 */
static inline void fromMsg(const whole_body_mpc_msgs::StateFeedbackGain &msg, Eigen::Ref<Eigen::MatrixXd> K) {
  if (K.rows() != msg.nu || K.cols() != msg.nx) {
    throw std::invalid_argument("The dimensions of K need to be: (" + std::to_string(msg.nu) + ", " +
                                std::to_string(msg.nx) + ").");
  }
  if (msg.data.size() != msg.nu * msg.nx) {
    throw std::invalid_argument("Message incorrect - size of data does not match given dimensions (nu,nx)");
  }

  for (std::size_t i = 0; i < msg.nu; ++i) {
    for (std::size_t j = 0; j < msg.nx; ++j) {
      K(i, j) = msg.data[i * msg.nx + j];
    }
  }
}
}  // namespace whole_body_mpc_msgs

#endif  // WHOLE_BODY_MPC_MSGS_CONVERSIONS_H_
