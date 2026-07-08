/*
 * @file double_integrator.hpp
 * @brief 1-D double integrator: the analytic benchmark model.
 *
 * State [p, v], control [a], forward Euler. Its discrete LQR solution is
 * computable in closed form, which makes it the cross-validation oracle for
 * the MPPI implementation (see test_mppi_control.cpp).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_DOUBLE_INTEGRATOR_HPP
#define XMNAV_MODELS_DOUBLE_INTEGRATOR_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {

struct DoubleIntegratorModel {
  static constexpr int kStateDim = 2;
  static constexpr int kControlDim = 1;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    State next;
    next(0) = x(0) + x(1) * dt;
    next(1) = x(1) + u(0) * dt;
    return next;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_DOUBLE_INTEGRATOR_HPP
