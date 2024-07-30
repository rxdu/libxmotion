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
      const Eigen::Matrix<double, 3, 4>& p_ddot,
      const Eigen::Matrix<double, 3, 4>& omega_dot, const Quaterniond& quat,
      const Eigen::Matrix<double, 3, 4>& foot_pos,
      const Eigen::Vector4d& foot_contact);

 private:
  ControllerParams::BalanceControllerParams params_;
  std::shared_ptr<QuadrupedModel> robot_model_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_BALANCE_CONTROLLER_HPP