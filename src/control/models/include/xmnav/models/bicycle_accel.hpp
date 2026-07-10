/*
 * @file bicycle_accel.hpp
 * @brief Kinematic bicycle with acceleration input.
 *
 * State [x, y, v, theta] (m, m, m/s, rad), control [a, delta] (m/s^2,
 * rad): the model the reachability Monte-Carlo simulation propagates
 * (successor of the retired model/BicycleKinematics). Exposes both the
 * continuous derivative (for RK4, see rk4.hpp) and the discrete Step
 * concept the rest of the stack uses (forward Euler).
 *
 * Equations, conventions, parameters, and validation oracles:
 * docs/typst/models.typ (compiled: models.pdf).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_BICYCLE_ACCEL_HPP
#define XMNAV_MODELS_BICYCLE_ACCEL_HPP

#include <cmath>

#include <eigen3/Eigen/Dense>

namespace xmotion {

struct BicycleAccelModel {
  static constexpr int kStateDim = 4;
  static constexpr int kControlDim = 2;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  double wheelbase = 2.4;  // m (the retired BicycleKinematics constant)

  State Deriv(const State &x, const Control &u) const {
    State xd;
    xd(0) = x(2) * std::cos(x(3));
    xd(1) = x(2) * std::sin(x(3));
    xd(2) = u(0);
    xd(3) = x(2) / wheelbase * std::tan(u(1));
    return xd;
  }

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    return x + Deriv(x, u) * dt;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_BICYCLE_ACCEL_HPP
