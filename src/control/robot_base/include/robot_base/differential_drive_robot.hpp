/*
 * @file differential_drive_robot.hpp
 * @date 4/21/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_
#define XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_

#include <vector>

#include "robot_base/speed_actuator_group.hpp"

namespace xmotion {
class DifferentialDriveRobot {
 public:
  DifferentialDriveRobot(std::shared_ptr<SpeedActuatorGroup> left,
                         std::shared_ptr<SpeedActuatorGroup> right);

  // public interface
  void SetMotionCommand(double linear_vel, double angular_vel);
  void GetMotionStatus(double &linear_vel, double &angular_vel);

 private:
  std::shared_ptr<SpeedActuatorGroup> left_;
  std::shared_ptr<SpeedActuatorGroup> right_;
};
}  // namespace xmotion

#endif  // XMOTION_DIFFERENTIAL_DRIVE_ROBOT_HPP_
