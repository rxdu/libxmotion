/*
 * @file balance_controller.cpp
 * @date 7/24/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/controller/balance_controller.hpp"

#include "quadprog++/QuadProg++.hh"
#include "math_utils/matrix.hpp"

namespace xmotion {
BalanceController::BalanceController(
    const ControllerParams::BalanceControllerParams& params,
    std::shared_ptr<QuadrupedModel> robot_model) {}

Eigen::Matrix<double, 3, 4> BalanceController::ComputeFootForce(
    const Eigen::Matrix<double, 3, 4>& p_ddot,
    const Eigen::Matrix<double, 3, 4>& omega_dot, const Quaterniond& quat,
    const Eigen::Matrix<double, 3, 4>& foot_pos,
    const Eigen::Vector4d& foot_contact) {
  Eigen::Matrix<double, 3, 4> f_d;

  // construct A
  Eigen::Matrix<double, 6, 12> A = Eigen::Matrix<double, 6, 12>::Zero();
  for (int i = 0; i < 4; ++i) {
    A.block<3, 3>(0, i * 3) = Eigen::Matrix<double, 3, 3>::Identity();
    //    Position3d p_g =
    //        foot_pos.col(i) -
    //        quat.toRotationMatrix() * robot_model_->GetFootPosInBodyFrame(i);
    //    A.block<3, 3>(3, i * 3) = MathUtils::SkewSymmetric(foot_pos.col(i));
  }
}
}  // namespace xmotion