/*
 * @file swerve_drive_kinematics.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/kinematics/swerve_drive_kinematics.hpp"

#include <iostream>

namespace xmotion {
SwerveDriveKinematics::SwerveDriveKinematics(
    const SwerveDriveKinematics::Param& param)
    : param_(param) {
  rx_ = param_.wheel_base / 2.0;
  ry_ = param_.track_width / 2.0;

  // clang-format off
  coeff_matrix_ <<
      1, 0, rx_,
      0, 1, ry_,
      1, 0, -rx_,
      0, 1, ry_,
      1, 0, -rx_,
      0, 1, -ry_,
      1, 0, rx_,
      0, 1, -ry_;
  // clang-format on
}

SwerveDriveKinematics::Commands SwerveDriveKinematics::ComputeWheelCommands(
    const Twist& twist) {
  Commands cmd;
  Eigen::Vector3d v_body = {twist.linear.x(), twist.linear.y(),
                            twist.angular.z()};
  Eigen::VectorXd v_wheel = coeff_matrix_ * v_body;
  std::cout << "v_wheel: \n" << v_wheel << std::endl;

  for (int i = 0; i < 4; i++) {
    cmd.angles[i] = std::atan2(v_wheel[i * 2 + 1], v_wheel[i * 2]);
    cmd.speeds[i] = std::sqrt(v_wheel[i * 2] * v_wheel[i * 2] +
                              v_wheel[i * 2 + 1] * v_wheel[i * 2 + 1]) /
                    param_.wheel_radius;
  }

  return cmd;
}
}  // namespace xmotion