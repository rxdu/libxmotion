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
RotationMatrix3d RollPitchYawToRotationMatrix(double roll, double pitch,
                                              double yaw);
HomogeneousMatrix3d CreateHomogeneousMatrix(const RotationMatrix3d& R,
                                            const Position3d& p);
HomogeneousMatrix3d GetHomogeneousMatrixInverse(const HomogeneousMatrix3d& in);
Position3d ApplyHomogeneousMatrix(const HomogeneousMatrix3d& H,
                                  const Position3d& p);
}  // namespace MatrixHelper
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_MATRIX_HELPER_HPP
