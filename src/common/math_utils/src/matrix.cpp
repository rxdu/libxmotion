/*
 * @file matrix.cpp
 * @date 7/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "math_utils/matrix.hpp"

namespace xmotion {
Eigen::Matrix<double, 3, 3> MathUtils::SkewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix<double, 3, 3> m;
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}
}  // namespace xmotion