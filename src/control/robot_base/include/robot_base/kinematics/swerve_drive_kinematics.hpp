/*
 * @file swerve_drive_kinematics.hpp
 * @date 11/14/24
 * @brief
 *
 * Reference:
 * [1]
 * https://www.freshconsulting.com/insights/blog/how-to-build-a-swerve-drive-robot/
 * [2]
 * https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SWERVE_DRIVE_KINEMATICS_HPP
#define XMOTION_SWERVE_DRIVE_KINEMATICS_HPP

#include <array>

#include <eigen3/Eigen/Core>

#include "interface/type/geometry_types.hpp"

namespace xmotion {
class SwerveDriveKinematics {
  enum SteeringRange {
    kPI = 0,
    k2PI,
  };

 public:
  struct Param {
    double track_width;   // d
    double wheel_base;    // l
    double wheel_radius;  // r

    double max_driving_speed;
    double max_steering_angle;

    double linear_vel_deadband = 0.005;
    double angular_vel_deadband = 0.005;
  };

  enum MotorIndex {
    kFrontRight = 0,
    kFrontLeft = 1,
    kRearLeft = 2,
    kRearRight = 3
  };

  struct Command {
    std::array<float, 4> speeds;
    std::array<float, 4> angles;
  };

  struct State {
    std::array<float, 4> speeds;
    std::array<float, 4> angles;
  };

 public:
  SwerveDriveKinematics() = default;
  SwerveDriveKinematics(const Param& param);

  // forward kinematics
  Command ComputeWheelCommands(const Twist& twist);

  // inverse kinematics

 private:
  Param param_;
  SteeringRange steering_range_ = SteeringRange::kPI;

  double rx_;
  double ry_;
  Eigen::Matrix<double, 8, 3> coeff_matrix_;
};
}  // namespace xmotion

#endif  // XMOTION_SWERVE_DRIVE_KINEMATICS_HPP