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

}  // namespace critic_core
}  // namespace xmotion

#endif  // XMNAV_MPPI_CRITIC_CORE_HPP
