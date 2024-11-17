/*
 * @file differential_drive_robot.hpp
 * @date 4/21/24
 * @brief
 *
 * Reference: https://en.wikipedia.org/wiki/Differential_wheeled_robot
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_
#define XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_

#include <vector>

#include "robot_base/actuator/speed_actuator_group.hpp"

namespace xmotion {
class DifferentialDriveRobot {
 public:
  struct Config {
    double track_width;   // d
    double wheel_radius;  // r

    double linear_vel_deadband;
    double angular_vel_deadband;

    bool reverse_left_wheel = false;
    bool reverse_right_wheel = false;

    std::shared_ptr<SpeedActuatorGroup> left_actuator_group;
    std::shared_ptr<SpeedActuatorGroup> right_actuator_group;
  };

 public:
  DifferentialDriveRobot(const Config &config);

  // public interface
  void SetMotionCommand(double linear_vel, double angular_vel);
  void GetMotionState(double &linear_vel, double &angular_vel);

 private:
  Config config_;
};
}  // namespace xmotion

#endif  // XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_
