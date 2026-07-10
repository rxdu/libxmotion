/*
 * @file cartpole.hpp
 * @brief Cart-pole (inverted pendulum on a cart).
 *
 * The classical underactuated benchmark, in the formulation of Barto,
 * Sutton & Anderson, "Neuronlike adaptive elements that can solve
 * difficult learning control problems", IEEE T-SMC 1983 (uniform pole,
 * frictionless). Default parameters are the literature-standard set
 * (1 kg cart, 0.1 kg pole, 0.5 m half-length).
 *
 * State [x (m), x_dot (m/s), theta (rad), theta_dot (rad/s)] with theta
 * measured FROM UPRIGHT (theta = 0 is the unstable equilibrium, theta =
 * pi hangs down); control [force (N)].
 *
 * Equations, conventions, parameters, and validation oracles:
 * docs/typst/models.typ (compiled: models.pdf).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_CARTPOLE_HPP
#define XMNAV_MODELS_CARTPOLE_HPP

#include <cmath>

#include <eigen3/Eigen/Dense>

#include "xmnav/models/model_core.hpp"

namespace xmotion {

struct CartPoleModel {
  static constexpr int kStateDim = 4;
  static constexpr int kControlDim = 1;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  double cart_mass = 1.0;         // kg
  double pole_mass = 0.1;         // kg
  double pole_half_length = 0.5;  // m, pivot to pole CoM
  double gravity = 9.81;

  State Deriv(const State &x, const Control &u) const {
    State xd;
    model_core::CartPoleDeriv(x.data(), u.data(), cart_mass, pole_mass,
                              pole_half_length, gravity, xd.data());
    return xd;
  }

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    return x + Deriv(x, u) * dt;
  }

  // total mechanical energy (uniform pole: I_com = m (2l)^2 / 12); zero
  // potential at the upright pole. Conserved when unforced — the test
  // oracle for the dynamics.
  double Energy(const State &x) const {
    const double l = pole_half_length;
    const double vpx = x(1) + l * x(3) * std::cos(x(2));
    const double vpy = -l * x(3) * std::sin(x(2));
    const double i_com = pole_mass * (2.0 * l) * (2.0 * l) / 12.0;
    return 0.5 * cart_mass * x(1) * x(1) +
           0.5 * pole_mass * (vpx * vpx + vpy * vpy) +
           0.5 * i_com * x(3) * x(3) +
           pole_mass * gravity * l * (std::cos(x(2)) - 1.0);
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_CARTPOLE_HPP
