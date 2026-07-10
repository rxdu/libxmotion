/*
 * @file model_core.hpp
 * @brief Raw-span dynamics cores shared by CPU and CUDA rollout backends.
 *
 * One implementation of each model's step math, templated on the scalar
 * (double on CPU, float on the GPU) and free of Eigen/STL so nvcc compiles
 * it with a minimal include surface. The Eigen model classes wrap these;
 * device programs call them directly (see docs/typst/mppi.typ).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_MODEL_CORE_HPP
#define XMNAV_MODELS_MODEL_CORE_HPP

#include <cmath>

#include "xmnav/models/device.hpp"

namespace xmotion {
namespace model_core {

// kinematic differential drive: state [x, y, theta], control [v, w],
// forward Euler
template <typename Scalar>
XMNAV_HD inline void DiffDriveStep(const Scalar x[3], const Scalar u[2],
                                   Scalar dt, Scalar next[3]) {
  next[0] = x[0] + u[0] * std::cos(x[2]) * dt;
  next[1] = x[1] + u[0] * std::sin(x[2]) * dt;
  next[2] = x[2] + u[1] * dt;
}

// --- quaternion helpers (w, x, y, z convention, unit quaternions) ---

template <typename Scalar>
XMNAV_HD inline void QuatNormalize(Scalar q[4]) {
  const Scalar n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                             q[3] * q[3]);
  const Scalar inv = Scalar(1) / n;
  q[0] *= inv;
  q[1] *= inv;
  q[2] *= inv;
  q[3] *= inv;
}

// Hamilton product out = a * b
template <typename Scalar>
XMNAV_HD inline void QuatMultiply(const Scalar a[4], const Scalar b[4],
                                  Scalar out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// rotation matrix of a unit quaternion, row-major R[3*r + c]
template <typename Scalar>
XMNAV_HD inline void QuatToRotation(const Scalar q[4], Scalar R[9]) {
  const Scalar w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z);
  R[1] = 2 * (x * y - w * z);
  R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z);
  R[4] = 1 - 2 * (x * x + z * z);
  R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y);
  R[7] = 2 * (y * z + w * x);
  R[8] = 1 - 2 * (x * x + y * y);
}

// Single-rigid-body quadruped step (see models/srb_quadruped.hpp for the
// layout and physics). Inputs beyond the state/control: world-frame foot
// positions at this step (flattened, 4 x 3), the stance mask, trunk
// parameters, dt. The world-inertia solve uses I_w^{-1} = R D^{-1} R^T
// exactly (D = diag inertia), so no factorization is needed.
//
// State (13): [p(3), v(3), q(4, wxyz body->world), omega(3, world frame)]
// Control (12): per-foot world-frame ground reaction forces.
template <typename Scalar>
XMNAV_HD inline void SrbQuadrupedStep(const Scalar x[13], const Scalar u[12],
                                      const Scalar feet[12],
                                      const unsigned char stance[4],
                                      const Scalar inertia_diag[3],
                                      Scalar mass, Scalar gravity, Scalar dt,
                                      Scalar next[13]) {
  const Scalar *p = x;
  const Scalar *v = x + 3;
  Scalar q[4] = {x[6], x[7], x[8], x[9]};
  QuatNormalize(q);
  const Scalar *omega = x + 10;

  // total force and torque about the CoM from stance feet only
  Scalar force_sum[3] = {0, 0, 0};
  Scalar torque_sum[3] = {0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    if (!stance[i]) continue;  // swing feet transmit nothing
    const Scalar *f = u + 3 * i;
    const Scalar r[3] = {feet[3 * i] - p[0], feet[3 * i + 1] - p[1],
                         feet[3 * i + 2] - p[2]};
    force_sum[0] += f[0];
    force_sum[1] += f[1];
    force_sum[2] += f[2];
    torque_sum[0] += r[1] * f[2] - r[2] * f[1];
    torque_sum[1] += r[2] * f[0] - r[0] * f[2];
    torque_sum[2] += r[0] * f[1] - r[1] * f[0];
  }

  Scalar R[9];
  QuatToRotation(q, R);

  // I_w omega (I_w = R D R^T): t1 = R^T omega, t2 = D t1, Iw_omega = R t2
  Scalar t1[3], t2[3], Iw_omega[3];
  for (int r = 0; r < 3; ++r) {
    t1[r] = R[r] * omega[0] + R[3 + r] * omega[1] + R[6 + r] * omega[2];
  }
  for (int r = 0; r < 3; ++r) t2[r] = inertia_diag[r] * t1[r];
  for (int r = 0; r < 3; ++r) {
    Iw_omega[r] = R[3 * r] * t2[0] + R[3 * r + 1] * t2[1] + R[3 * r + 2] * t2[2];
  }

  // rhs = torque - omega x (I_w omega); omega_dot = R D^{-1} R^T rhs
  const Scalar rhs[3] = {
      torque_sum[0] - (omega[1] * Iw_omega[2] - omega[2] * Iw_omega[1]),
      torque_sum[1] - (omega[2] * Iw_omega[0] - omega[0] * Iw_omega[2]),
      torque_sum[2] - (omega[0] * Iw_omega[1] - omega[1] * Iw_omega[0])};
  Scalar s1[3], s2[3], omega_dot[3];
  for (int r = 0; r < 3; ++r) {
    s1[r] = R[r] * rhs[0] + R[3 + r] * rhs[1] + R[6 + r] * rhs[2];
  }
  for (int r = 0; r < 3; ++r) s2[r] = s1[r] / inertia_diag[r];
  for (int r = 0; r < 3; ++r) {
    omega_dot[r] = R[3 * r] * s2[0] + R[3 * r + 1] * s2[1] + R[3 * r + 2] * s2[2];
  }

  // integrate (forward Euler; quaternion via the world-frame omega increment)
  const Scalar half_dt = Scalar(0.5) * dt;
  const Scalar dq[4] = {Scalar(1), half_dt * omega[0], half_dt * omega[1],
                        half_dt * omega[2]};
  Scalar q_next[4];
  QuatMultiply(dq, q, q_next);
  QuatNormalize(q_next);

  for (int i = 0; i < 3; ++i) next[i] = p[i] + v[i] * dt;
  next[3] = v[0] + (force_sum[0] / mass) * dt;
  next[4] = v[1] + (force_sum[1] / mass) * dt;
  next[5] = v[2] + (force_sum[2] / mass - gravity) * dt;
  for (int i = 0; i < 4; ++i) next[6 + i] = q_next[i];
  for (int i = 0; i < 3; ++i) next[10 + i] = omega[i] + omega_dot[i] * dt;
}

// Cart-pole (inverted pendulum on a cart), the classical underactuated
// benchmark, in the exact formulation of Barto, Sutton & Anderson 1983
// (uniform pole, frictionless; pole_half_length is l, the pivot-to-CoM
// distance). State [x, x_dot, theta, theta_dot] with theta measured FROM
// UPRIGHT (theta = 0 is the unstable equilibrium); control [force].
template <typename Scalar>
XMNAV_HD inline void CartPoleDeriv(const Scalar x[4], const Scalar u[1],
                                   Scalar cart_mass, Scalar pole_mass,
                                   Scalar pole_half_length, Scalar gravity,
                                   Scalar xd[4]) {
  const Scalar st = std::sin(x[2]);
  const Scalar ct = std::cos(x[2]);
  const Scalar total = cart_mass + pole_mass;
  const Scalar tmp =
      (u[0] + pole_mass * pole_half_length * x[3] * x[3] * st) / total;
  const Scalar theta_dd =
      (gravity * st - ct * tmp) /
      (pole_half_length *
       (Scalar(4.0 / 3.0) - pole_mass * ct * ct / total));
  const Scalar x_dd =
      tmp - pole_mass * pole_half_length * theta_dd * ct / total;
  xd[0] = x[1];
  xd[1] = x_dd;
  xd[2] = x[3];
  xd[3] = theta_dd;
}

// Single-track ("bicycle") model with linear tires, Rajamani ch. 2:
// body-frame lateral dynamics with front/rear cornering stiffness. State
// [X, Y, psi, vx, vy, r] (world position/heading, body velocities, yaw
// rate); control [ax, delta] (longitudinal acceleration command, front
// steering angle). Denominators use max(vx, vx_min) — the linear tire
// model is meaningless near standstill.
template <typename Scalar>
XMNAV_HD inline void DynamicBicycleDeriv(
    const Scalar x[6], const Scalar u[2], Scalar mass, Scalar yaw_inertia,
    Scalar lf, Scalar lr, Scalar cornering_front, Scalar cornering_rear,
    Scalar vx_min, Scalar xd[6]) {
  const Scalar vx = x[3] > vx_min ? x[3] : vx_min;
  const Scalar alpha_f = (x[4] + lf * x[5]) / vx - u[1];
  const Scalar alpha_r = (x[4] - lr * x[5]) / vx;
  const Scalar fyf = -cornering_front * alpha_f;
  const Scalar fyr = -cornering_rear * alpha_r;
  const Scalar cd = std::cos(u[1]);
  const Scalar cp = std::cos(x[2]);
  const Scalar sp = std::sin(x[2]);
  xd[0] = x[3] * cp - x[4] * sp;
  xd[1] = x[3] * sp + x[4] * cp;
  xd[2] = x[5];
  xd[3] = u[0] + x[4] * x[5];
  xd[4] = (fyf * cd + fyr) / mass - x[3] * x[5];
  xd[5] = (lf * fyf * cd - lr * fyr) / yaw_inertia;
}

// Quadrotor rigid-body dynamics, Mellinger & Kumar 2011. State
// [p(3), v(3), q(4, wxyz body->world), omega(3, BODY frame — note the
// SRB quadruped uses world-frame omega)]; control [thrust, tau(3)]
// (total thrust along body z, body-frame moments). Derivative of the
// quaternion is returned in xd[6..9] (q_dot = 0.5 q (x) (0, omega));
// integrators must renormalize q after stepping.
template <typename Scalar>
XMNAV_HD inline void QuadrotorDeriv(const Scalar x[13], const Scalar u[4],
                                    Scalar mass,
                                    const Scalar inertia_diag[3],
                                    Scalar gravity, Scalar xd[13]) {
  const Scalar *q = x + 6;
  const Scalar *w = x + 10;
  Scalar R[9];
  QuatToRotation(q, R);
  // v_dot = -g e3 + (thrust/m) R e3
  xd[0] = x[3];
  xd[1] = x[4];
  xd[2] = x[5];
  xd[3] = (u[0] / mass) * R[2];
  xd[4] = (u[0] / mass) * R[5];
  xd[5] = (u[0] / mass) * R[8] - gravity;
  // q_dot = 0.5 q (x) (0, omega_body)
  const Scalar half_w[4] = {Scalar(0), Scalar(0.5) * w[0],
                            Scalar(0.5) * w[1], Scalar(0.5) * w[2]};
  QuatMultiply(q, half_w, xd + 6);
  // omega_dot = I^-1 (tau - omega x I omega)
  const Scalar Iw[3] = {inertia_diag[0] * w[0], inertia_diag[1] * w[1],
                        inertia_diag[2] * w[2]};
  xd[10] = (u[1] - (w[1] * Iw[2] - w[2] * Iw[1])) / inertia_diag[0];
  xd[11] = (u[2] - (w[2] * Iw[0] - w[0] * Iw[2])) / inertia_diag[1];
  xd[12] = (u[3] - (w[0] * Iw[1] - w[1] * Iw[0])) / inertia_diag[2];
}

}  // namespace model_core
}  // namespace xmotion

#endif  // XMNAV_MODELS_MODEL_CORE_HPP
