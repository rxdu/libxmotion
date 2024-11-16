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
namespace {
template <typename T>
int sign(T val) {
  return (T(0) < val) - (val < T(0));
}
}  // namespace

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

  if (param_.max_steering_angle > M_PI / 2.0) {
    steering_range_ = SteeringRange::k2PI;
  } else {
    steering_range_ = SteeringRange::kPI;
  }
}

SwerveDriveKinematics::Command SwerveDriveKinematics::ComputeWheelCommands(
    const Twist& twist) {
  Command cmd;
  Eigen::Vector3d v_body = {twist.linear.x(), twist.linear.y(),
                            twist.angular.z()};
  Eigen::VectorXd v_wheel = coeff_matrix_ * v_body;

  for (int i = 0; i < 4; i++) {
    // calculate steering angle
    if (std::abs(v_wheel[i * 2]) < param_.linear_vel_deadband &&
        std::abs(v_wheel[i * 2 + 1]) < param_.linear_vel_deadband) {
      cmd.angles[i] = 0;
    } else {
      if (steering_range_ == SteeringRange::kPI) {
        cmd.angles[i] = std::atan(v_wheel[i * 2 + 1] / v_wheel[i * 2]);
      } else {
        cmd.angles[i] = std::atan2(v_wheel[i * 2 + 1], v_wheel[i * 2]);
      }
    }

    // calculate driving speed
    Eigen::Vector2d body_v_vec(twist.linear.x(), twist.linear.y());
    Eigen::Vector2d wheel_dir_vec =
        Eigen::Vector2d(std::cos(cmd.angles[i]), std::sin(cmd.angles[i]));
    int v_sign = body_v_vec.dot(wheel_dir_vec) > 0 ? 1 : -1;
    cmd.speeds[i] = v_sign *
                    std::sqrt(v_wheel[i * 2] * v_wheel[i * 2] +
                              v_wheel[i * 2 + 1] * v_wheel[i * 2 + 1]) /
                    param_.wheel_radius;
  }

  return cmd;
}
}  // namespace xmotion