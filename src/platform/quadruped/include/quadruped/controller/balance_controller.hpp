/*
 * @file balance_controller.hpp
 * @date 7/24/24
 * @brief a simple balance controller from unitree book
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_BALANCE_CONTROLLER_HPP
#define QUADRUPED_MOTION_BALANCE_CONTROLLER_HPP

#include "quadruped/system_config.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
class BalanceController {
 public:
  BalanceController(const ControllerParams::BalanceControllerParams& params,
                    std::shared_ptr<QuadrupedModel> robot_model);

  Eigen::Matrix<double, 3, 4> ComputeFootForce(
      double mu, const Eigen::Matrix<double, 3, 1>& p_ddot,
      const Eigen::Matrix<double, 3, 1>& omega_dot, const Quaterniond& quat,
      const Eigen::Matrix<double, 3, 4>& foot_pos,
      const Eigen::Vector4d& foot_contact);

 private:
  const Eigen::Vector3d g_const = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Matrix<double, 12, 1> f_prev_;
  ControllerParams::BalanceControllerParams params_;
  std::shared_ptr<QuadrupedModel> robot_model_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_BALANCE_CONTROLLER_HPP