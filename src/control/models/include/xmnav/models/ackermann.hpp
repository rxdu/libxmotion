/*
 * @file ackermann.hpp
 * @brief Kinematic Ackermann (bicycle) model for MPPI rollouts.
 *
 * State [x, y, theta], control [v, delta] (speed, steering angle), wheelbase
 * L, forward Euler: thetadot = v/L * tan(delta).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_ACKERMANN_HPP
#define XMNAV_MODELS_ACKERMANN_HPP

#include <cmath>

#include <eigen3/Eigen/Dense>

namespace xmotion {

struct AckermannModel {
  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  double wheelbase = 0.5;  // m

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    State next;
    next(0) = x(0) + u(0) * std::cos(x(2)) * dt;
    next(1) = x(1) + u(0) * std::sin(x(2)) * dt;
    next(2) = x(2) + u(0) / wheelbase * std::tan(u(1)) * dt;
    return next;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_ACKERMANN_HPP
