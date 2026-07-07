/*
 * @file critic_core.hpp
 * @brief Raw-span/scalar cost cores shared by CPU and CUDA rollout backends.
 *
 * One implementation of each cost formula, templated on the scalar (double
 * on CPU, float on the GPU) and free of Eigen/STL so nvcc compiles it with
 * a minimal include surface. The Eigen critic classes wrap these; device
 * programs call them directly (see docs/typst/mppi.typ).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CRITIC_CORE_HPP
#define XMNAV_MPPI_CRITIC_CORE_HPP

#include <cmath>

#include "xmnav/mppi/device.hpp"

namespace xmotion {
namespace critic_core {

// SE(2) goal: quadratic position error + (1 - cos) heading error (wrap-free)
template <typename Scalar>
XMNAV_HD inline Scalar Se2GoalStage(const Scalar x[3], const Scalar goal[3],
                                    Scalar position_weight,
                                    Scalar heading_weight) {
  const Scalar dx = x[0] - goal[0];
  const Scalar dy = x[1] - goal[1];
  return position_weight * (dx * dx + dy * dy) +
         heading_weight * (Scalar(1) - std::cos(x[2] - goal[2]));
}

// soft penalty of one circular obstacle given the planar offset to its
// center; zero outside radius + margin
template <typename Scalar>
XMNAV_HD inline Scalar CircularObstaclePenalty(Scalar dx, Scalar dy,
                                               Scalar radius, Scalar margin,
                                               Scalar weight) {
  const Scalar clearance = std::sqrt(dx * dx + dy * dy) - radius - margin;
  return clearance < Scalar(0) ? weight * clearance * clearance : Scalar(0);
}

// SRB trunk tracking (critics_srb.hpp): height, tilt of the body z axis,
// world-velocity error, angular-rate damping. State layout of
// models/srb_quadruped.hpp; q is unit (normalized by the model step).
template <typename Scalar>
XMNAV_HD inline Scalar SrbTrackingStage(const Scalar x[13], Scalar height_ref,
                                        const Scalar velocity_ref[3],
                                        Scalar height_weight,
                                        Scalar tilt_weight,
                                        Scalar velocity_weight,
                                        Scalar angular_rate_weight) {
  const Scalar dh = x[2] - height_ref;
  // z-component of the rotated body z axis: R(2,2) of the unit quaternion
  const Scalar body_z_z =
      Scalar(1) - Scalar(2) * (x[7] * x[7] + x[8] * x[8]);
  const Scalar tilt = Scalar(1) - body_z_z;
  Scalar dv_sq = 0;
  for (int i = 0; i < 3; ++i) {
    const Scalar dv = x[3 + i] - velocity_ref[i];
    dv_sq += dv * dv;
  }
  const Scalar w_sq =
      x[10] * x[10] + x[11] * x[11] + x[12] * x[12];
  return height_weight * dh * dh + tilt_weight * tilt +
         velocity_weight * dv_sq + angular_rate_weight * w_sq;
}

// friction-cone soft penalty of one stance-foot force: f_z >= 0 and
// |f_xy| <= mu * f_z (critics_srb.hpp)
template <typename Scalar>
XMNAV_HD inline Scalar FrictionConePenalty(const Scalar f[3], Scalar mu,
                                           Scalar weight) {
  const Scalar zero = Scalar(0);
  const Scalar pull = f[2] < zero ? -f[2] : zero;
  const Scalar fz_pos = f[2] > zero ? f[2] : zero;
  const Scalar fxy = std::sqrt(f[0] * f[0] + f[1] * f[1]);
  const Scalar over = fxy - mu * fz_pos;
  const Scalar slip = over > zero ? over : zero;
  return weight * (pull * pull + slip * slip);
}

}  // namespace critic_core
}  // namespace xmotion

#endif  // XMNAV_MPPI_CRITIC_CORE_HPP
