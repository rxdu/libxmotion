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

#ifndef XMNAV_MPPI_MODEL_CORE_HPP
#define XMNAV_MPPI_MODEL_CORE_HPP

#include <cmath>

#include "xmnav/mppi/device.hpp"

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

}  // namespace model_core
}  // namespace xmotion

#endif  // XMNAV_MPPI_MODEL_CORE_HPP
