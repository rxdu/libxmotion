/*
 * matrix_helper.cpp
 *
 * Created on 7/16/24 10:29 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/matrix_helper.hpp"

#include <cmath>

namespace xmotion {
namespace {
RotationMatrix3d RotX(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotationMatrix3d R;
  R << 1, 0, 0, 0, c, -s, 0, s, c;
  return R;
}

RotationMatrix3d RotY(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotationMatrix3d R;
  R << c, 0, s, 0, 1, 0, -s, 0, c;
  return R;
}

RotationMatrix3d RotZ(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotationMatrix3d R;
  R << c, -s, 0, s, c, 0, 0, 0, 1;
  return R;
}
}  // namespace

RotationMatrix3d MatrixHelper::RollPitchYawToRotationMatrix(double roll,
                                                            double pitch,
                                                            double yaw) {
  return RotZ(yaw) * RotY(pitch) * RotX(roll);
}

HomogeneousMatrix3d MatrixHelper::CreateHomogeneousMatrix(
    const RotationMatrix3d& R, const Position3d& p) {
  HomogeneousMatrix3d H;
  H.block<3, 3>(0, 0) = R;
  H.block<3, 1>(0, 3) = p;
  H.block<1, 4>(3, 0) << 0, 0, 0, 1;
  return H;
}

HomogeneousMatrix3d MatrixHelper::GetHomogeneousMatrixInverse(
    const HomogeneousMatrix3d& in) {
  HomogeneousMatrix3d out;
  out.block<3, 3>(0, 0) = in.topLeftCorner(3, 3).transpose();
  out.block<3, 1>(0, 3) =
      -in.topLeftCorner(3, 3).transpose() * in.topRightCorner(3, 1);
  out.block<1, 4>(3, 0) << 0, 0, 0, 1;
  return out;
}

Position3d MatrixHelper::ApplyHomogeneousMatrix(const HomogeneousMatrix3d& H,
                                                const Position3d& p) {
  return H.block<3, 3>(0, 0) * p + H.block<3, 1>(0, 3);
}
}  // namespace xmotion