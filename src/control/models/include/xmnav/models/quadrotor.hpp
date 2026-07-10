/*
 * @file quadrotor.hpp
 * @brief Quadrotor rigid-body dynamics.
 *
 * The canonical aerial benchmark (Mellinger & Kumar, "Minimum snap
 * trajectory generation and control for quadrotors", ICRA 2011). Default
 * parameters are a Kumar-lab-scale small quadrotor.
 *
 * State (13): [p(3) m, v(3) m/s, q(4, wxyz body->world),
 * omega(3) rad/s in the BODY frame — note the SRB quadruped model uses
 * world-frame omega]; control (4): [thrust (N, along body z),
 * tau(3) (N m, body frame)]. Step() renormalizes the quaternion.
 *
 * Equations, conventions, parameters, and validation oracles:
 * docs/typst/models.typ (compiled: models.pdf).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_QUADROTOR_HPP
#define XMNAV_MODELS_QUADROTOR_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "xmnav/models/model_core.hpp"

namespace xmotion {

struct QuadrotorModel {
  static constexpr int kStateDim = 13;
  static constexpr int kControlDim = 4;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  double mass = 0.5;  // kg
  Eigen::Vector3d inertia_diag{2.32e-3, 2.32e-3, 4.0e-3};  // kg m^2
  double gravity = 9.81;

  static State MakeState(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                         const Eigen::Quaterniond &q,
                         const Eigen::Vector3d &omega_body) {
    State x;
    x.segment<3>(0) = p;
    x.segment<3>(3) = v;
    x(6) = q.w();
    x(7) = q.x();
    x(8) = q.y();
    x(9) = q.z();
    x.segment<3>(10) = omega_body;
    return x;
  }
  static Eigen::Vector3d Position(const State &x) { return x.segment<3>(0); }
  static Eigen::Vector3d Velocity(const State &x) { return x.segment<3>(3); }
  static Eigen::Quaterniond Orientation(const State &x) {
    return Eigen::Quaterniond(x(6), x(7), x(8), x(9));
  }
  static Eigen::Vector3d BodyRates(const State &x) {
    return x.segment<3>(10);
  }

  // gravity-compensating hover thrust
  double HoverThrust() const { return mass * gravity; }

  State Deriv(const State &x, const Control &u) const {
    State xd;
    model_core::QuadrotorDeriv(x.data(), u.data(), mass,
                               inertia_diag.data(), gravity, xd.data());
    return xd;
  }

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    State next = x + Deriv(x, u) * dt;
    model_core::QuatNormalize(next.data() + 6);
    return next;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_QUADROTOR_HPP
