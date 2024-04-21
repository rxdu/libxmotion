/*
 * differential_drive_robot.cpp
 *
 * Created on 4/21/24 10:38 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/differential_drive_robot.hpp"

namespace xmotion {
DifferentialDriveRobot::DifferentialDriveRobot(
    std::shared_ptr<SpeedActuatorGroup> left,
    std::shared_ptr<SpeedActuatorGroup> right)
    : left_(left), right_(right) {}

void DifferentialDriveRobot::SetMotionCommand(double linear_vel,
                                              double angular_vel) {}

void DifferentialDriveRobot::GetMotionStatus(double& linear_vel,
                                             double& angular_vel) {}
}  // namespace xmotion