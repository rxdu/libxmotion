/*
 * @file dynamic_bicycle.hpp
 * @brief Single-track ("bicycle") model with linear tires.
 *
 * The standard vehicle lateral-dynamics benchmark (Rajamani, "Vehicle
 * Dynamics and Control", ch. 2; the kinematic-vs-dynamic comparison
 * model of Kong et al., IV 2015). Its role in this repo: the designated
 * MISMATCH PLANT — planners use the kinematic bicycle; this plant slips.
 * Default parameters are a mid-size passenger car (Rajamani's typical
 * values).
 *
 * State [X (m), Y (m), psi (rad), vx (m/s), vy (m/s), r (rad/s)] —
 * world position/heading, body-frame velocities, yaw rate; control
 * [ax (m/s^2), delta (rad)] — longitudinal acceleration command and
 * front steering angle. The linear tire model is meaningless near
 * standstill: slip-angle denominators clamp at vx_min.
 *
 * Equations, conventions, parameters, and validation oracles:
 * docs/typst/models.typ (compiled: models.pdf).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_DYNAMIC_BICYCLE_HPP
#define XMNAV_MODELS_DYNAMIC_BICYCLE_HPP

#include <eigen3/Eigen/Dense>

#include "xmnav/models/model_core.hpp"

namespace xmotion {

struct DynamicBicycleModel {
  static constexpr int kStateDim = 6;
  static constexpr int kControlDim = 2;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  double mass = 1500.0;             // kg
  double yaw_inertia = 2500.0;      // kg m^2
  double lf = 1.2;                  // m, CoM to front axle
  double lr = 1.6;                  // m, CoM to rear axle
  double cornering_front = 80000.0; // N/rad, per axle
  double cornering_rear = 80000.0;  // N/rad, per axle
  double vx_min = 0.5;              // m/s, tire-model validity clamp

  State Deriv(const State &x, const Control &u) const {
    State xd;
    model_core::DynamicBicycleDeriv(x.data(), u.data(), mass, yaw_inertia,
                                    lf, lr, cornering_front, cornering_rear,
                                    vx_min, xd.data());
    return xd;
  }

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    return x + Deriv(x, u) * dt;
  }

  // steady-state yaw rate under constant speed and steering (Rajamani):
  // r_ss = vx / (L + Kus vx^2) * delta, Kus = m/L (lr/Cf - lf/Cr) —
  // the closed-form test oracle
  double SteadyStateYawRate(double vx, double delta) const {
    const double L = lf + lr;
    const double kus =
        mass / L * (lr / cornering_front - lf / cornering_rear);
    return vx / (L + kus * vx * vx) * delta;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_DYNAMIC_BICYCLE_HPP
