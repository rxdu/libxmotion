/*
 * @file swerve_drive_robot.hpp
 * @date 11/9/24
 * @brief 4-wheel swerve drive robot control class
 *
 * Assumptions:
 * Motors/servos are numbered in the following order:
 * 1. Front right
 * 2. Front left
 * 3. Rear left
 * 4. Rear right
 *
 * Robot command:
 * - Steering angles: in radian [-M_PI/2.0f, M_PI/2.0f]
 * - Driving speeds: in m/s
 *
 * It's assumed that the underlying steering motor driver uses degree as unit.
 *
 * Robot coordinate system:
 * x: forward
 * y: left
 * z: up
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SWERVE_DRIVE_ROBOT_HPP
#define XMOTION_SWERVE_DRIVE_ROBOT_HPP

#include <vector>
#include <array>
#include <memory>

#include "interface/type/geometry_types.hpp"

#include "interface/driver/motor_controller_array_interface.hpp"

namespace xmotion {
class SwerveDriveRobot {
 public:
  struct Config {
    double track_width;   // d
    double wheel_base;    // l
    double wheel_radius;  // r

    bool reverse_left_wheels = false;
    bool reverse_right_wheels = false;

    std::shared_ptr<MotorControllerArrayInterface> driving_motors;
    std::shared_ptr<MotorControllerArrayInterface> steering_motors;
  };

 public:
  SwerveDriveRobot(const Config &config);

  // public interface
  void SetMotionCommand(const Twist &twist);
  void SetSteeringCommand(const std::array<float, 4> &angles);
  void SetDrivingCommand(const std::array<float, 4> &speeds);

 private:
  Config config_;
};
}  // namespace xmotion

#endif  // XMOTION_SWERVE_DRIVE_ROBOT_HPP