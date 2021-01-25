///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Oxford, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_
#define WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_

#include "whole_body_mpc_msgs/conversions.h"

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
   * @param[in] K  State feedback gain (size nu * nx)
   * @return The ROS message that contains the state feedback gain
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
   * @param[in] msg  ROS message that contains the state feedback gain
   * @return The state feedback gain
   */
  const Eigen::MatrixXd &writeFromMessage(const whole_body_mpc_msgs::StateFeedbackGain &msg) {
    if (msg.data.size() != nu_ * nx_ || msg.nu != nu_ || msg.nx != nx_) {
      throw std::invalid_argument("Dimensions of message do not match nx, nu of StateFeedbackGainInterface.");
    }
    fromMsg(msg, K_);
    return K_;
  }

 private:
  std::size_t nx_;                              //!< Dimension of the state vector
  std::size_t nu_;                              //!< Dimension of the control vector
  whole_body_mpc_msgs::StateFeedbackGain msg_;  //!< ROS message that contains the state feedback gain
  Eigen::MatrixXd K_;                           //!< State feedback gain
};
}  // namespace whole_body_mpc_msgs

#endif  // WHOLE_BODY_MPC_MSGS_STATE_FEEDBACK_GAIN_INTERFACE_H_
