/*
 * matrix_utils.hpp
 *
 * Small Eigen helpers for the estimation filters (MEKF attitude math).
 *
 * Repatriated from xmbase/math (ADR 0007 placement rule: one function,
 * one consumer component — a primitive with a single consumer lives with
 * that consumer, and the foundation stays Eigen-light).
 *
 * Copyright (c) 2024-2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_ESTIMATION_MATRIX_UTILS_HPP
#define XMNAV_ESTIMATION_MATRIX_UTILS_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {
namespace MathUtils {
inline Eigen::Matrix<double, 3, 3> SkewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix<double, 3, 3> m;
  // clang-format off
  m <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  // clang-format on
  return m;
}
}  // namespace MathUtils
}  // namespace xmotion

#endif  // XMNAV_ESTIMATION_MATRIX_UTILS_HPP
