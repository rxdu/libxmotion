/*
 * @file diff_drive.hpp
 * @brief Kinematic differential-drive model for MPPI rollouts.
 *
 * State [x, y, theta] (m, m, rad), control [v, w] (m/s, rad/s), forward
 * Euler. This is the model class production wheeled MPPI deployments use
 * (Nav2); dynamics effects are absorbed by the box constraints.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_MODELS_DIFF_DRIVE_HPP
#define XMNAV_MPPI_MODELS_DIFF_DRIVE_HPP

#include <cmath>

#include <eigen3/Eigen/Dense>

namespace xmotion {

struct DiffDriveModel {
  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  State Step(const State &x, const Control &u, double dt) const {
    State next;
    next(0) = x(0) + u(0) * std::cos(x(2)) * dt;
    next(1) = x(1) + u(0) * std::sin(x(2)) * dt;
    next(2) = x(2) + u(1) * dt;
    return next;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_MODELS_DIFF_DRIVE_HPP
