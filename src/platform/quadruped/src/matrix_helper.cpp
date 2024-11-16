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
RotMatrix3d RotX(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotMatrix3d R;
  R << 1, 0, 0, 0, c, -s, 0, s, c;
  return R;
}

RotMatrix3d RotY(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotMatrix3d R;
  R << c, 0, s, 0, 1, 0, -s, 0, c;
  return R;
}

RotMatrix3d RotZ(double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  RotMatrix3d R;
  R << c, -s, 0, s, c, 0, 0, 0, 1;
  return R;
}
}  // namespace

RotMatrix3d MatrixHelper::RpyToRotMatrix(double roll, double pitch,
                                         double yaw) {
  return RotZ(yaw) * RotY(pitch) * RotX(roll);
}

HomoMatrix3d MatrixHelper::CreateHomoMatrix(const RotMatrix3d& R,
                                            const Position3d& p) {
  HomoMatrix3d H;
  H.block<3, 3>(0, 0) = R;
  H.block<3, 1>(0, 3) = p;
  H.block<1, 4>(3, 0) << 0, 0, 0, 1;
  return H;
}

HomoMatrix3d MatrixHelper::GetHomoMatrixInverse(const HomoMatrix3d& in) {
  HomoMatrix3d out;
  out.block<3, 3>(0, 0) = in.topLeftCorner(3, 3).transpose();
  out.block<3, 1>(0, 3) =
      -in.topLeftCorner(3, 3).transpose() * in.topRightCorner(3, 1);
  out.block<1, 4>(3, 0) << 0, 0, 0, 1;
  return out;
}

Position3d MatrixHelper::ApplyHomoMatrix(const HomoMatrix3d& H,
                                         const Position3d& p) {
  return H.block<3, 3>(0, 0) * p + H.block<3, 1>(0, 3);
}

RotMatrix3d MatrixHelper::GetSkewSymmetricMatrix(const Eigen::Vector3d& m) {
  RotMatrix3d M;
  M << 0, -m(2), m(1), m(2), 0, -m(0), -m(1), m(0), 0;
  return M;
}

Eigen::Vector3d MatrixHelper::GetExponentialMap(const RotMatrix3d& rm) {
  double cos_value = rm.trace() / 2.0 - 1 / 2.0;
  if (cos_value > 1.0f) {
    cos_value = 1.0f;
  } else if (cos_value < -1.0f) {
    cos_value = -1.0f;
  }

  double angle = std::acos(cos_value);
  Eigen::Vector3d exp;
  if (std::fabs(angle) < 1e-5) {
    exp = Eigen::Vector3d(0, 0, 0);
  } else if (std::fabs(angle - M_PI) < 1e-5) {
    exp = angle * Eigen::Vector3d(rm(0, 0) + 1, rm(0, 1), rm(0, 2)) /
          std::sqrt(2 * (1 + rm(0, 0)));
  } else {
    exp = angle / (2.0f * std::sin(angle)) *
          Eigen::Vector3d(rm(2, 1) - rm(1, 2), rm(0, 2) - rm(2, 0),
                          rm(1, 0) - rm(0, 1));
  }
  return exp;
}
}  // namespace xmotion