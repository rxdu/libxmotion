/*
 * @file swerve_drive_robot.hpp
 * @date 11/9/24
 * @brief
 *
 * Assumptions:
 *  Steering angles: [-90, 90] degrees
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

    double linear_vel_deadband;
    double angular_vel_deadband;

    bool reverse_left_wheel = false;
    bool reverse_right_wheel = false;

    std::shared_ptr<MotorControllerArrayInterface> driving_motors;
    std::shared_ptr<MotorControllerArrayInterface> steering_motors;
  };

 public:
  SwerveDriveRobot(const Config &config);

  // public interface
  void SetMotionCommand(const Twist &twist);
  void SetActuatorCommand(const std::array<float, 4> &speeds,
                          const std::array<float, 4> &angles);

 private:
  Config config_;
};
}  // namespace xmotion

#endif  // XMOTION_SWERVE_DRIVE_ROBOT_HPP