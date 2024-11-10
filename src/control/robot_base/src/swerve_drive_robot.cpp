/*
 * @file swerve_drive_robot.cpp
 * @date 11/9/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/swerve_drive_robot.hpp"

namespace xmotion {
SwerveDriveRobot::SwerveDriveRobot(const SwerveDriveRobot::Config& config)
    : config_(config) {}

void SwerveDriveRobot::SetMotionCommand(const Twist& twist) {}

void SwerveDriveRobot::SetActuatorCommand(const std::array<float, 4>& speeds,
                                          const std::array<float, 4>& angles) {

}
}  // namespace xmotion