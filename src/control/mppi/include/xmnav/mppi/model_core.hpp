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

}  // namespace model_core
}  // namespace xmotion

#endif  // XMNAV_MPPI_MODEL_CORE_HPP
