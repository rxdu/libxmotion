/*
 * @file matrix_helper.hpp
 * @date 7/16/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_MATRIX_HELPER_HPP
#define QUADRUPED_MOTION_MATRIX_HELPER_HPP

#include "interface/type/geometry_types.hpp"

namespace xmotion {
namespace MatrixHelper {
/**
 * @brief Create a 3x3 rotation matrix from roll, pitch, and yaw angles
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
RotMatrix3d RpyToRotMatrix(double roll, double pitch, double yaw);

/**
 * @brief Create a homogeneous matrix from a rotation matrix and a position
 * vector
 * @param R
 * @param p
 * @return
 */
HomoMatrix3d CreateHomoMatrix(const RotMatrix3d& R, const Position3d& p);

/**
 * @brief Get the inverse of a homogeneous matrix
 * @param in
 * @return
 */
HomoMatrix3d GetHomoMatrixInverse(const HomoMatrix3d& in);

/**
 * @brief Apply a homogeneous matrix to a position vector
 * @param H
 * @param p
 * @return
 */
Position3d ApplyHomoMatrix(const HomoMatrix3d& H, const Position3d& p);

/**
 * @brief Get the skew symmetric matrix of a 3x1 vector
 * @param m
 * @return
 */
RotMatrix3d GetSkewSymmetricMatrix(const Eigen::Vector3d& m);

/**
 * @brief Get the exponential map of a rotation matrix
 * @param rm
 * @return
 */
Eigen::Vector3d GetExponentialMap(const RotMatrix3d& rm);
}  // namespace MatrixHelper
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_MATRIX_HELPER_HPP
