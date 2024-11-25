/*
 * @file test_swerve_drive_kinematics.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "robot_base/kinematics/swerve_drive_kinematics.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  SwerveDriveKinematics::Param param;
  param.track_width = 0.198;
  param.wheel_base = 0.245;
  param.wheel_radius = 0.0363;
  SwerveDriveKinematics kinematics(param);

  auto command = kinematics.ComputeWheelCommands({{1, 0.5, 0}, {0, 0, 0.2}});

  std::cout << "Computed wheel angles: ";
  for (auto angle : command.angles) std::cout << angle << " ";
  std::cout << std::endl;
  std::cout << "Computed wheel speeds: ";
  for (auto speed : command.speeds) std::cout << speed << " ";
  std::cout << std::endl;

  return 0;
}