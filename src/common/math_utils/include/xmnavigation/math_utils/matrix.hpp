/*
 * @file matrix.hpp
 * @date 7/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_MATRIX_HPP
#define QUADRUPED_MOTION_MATRIX_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {
namespace MathUtils {
Eigen::Matrix<double, 3, 3> SkewSymmetric(const Eigen::Vector3d &v);
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_MATRIX_HPP