/*
 * @file swerve_drive_robot.cpp
 * @date 11/9/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/swerve_drive_robot.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SwerveDriveRobot::SwerveDriveRobot(const SwerveDriveRobot::Config& config)
    : config_(config) {}

void SwerveDriveRobot::SetMotionCommand(const Twist& twist) {}

void SwerveDriveRobot::SetSteeringCommand(const std::array<float, 4>& angles) {
  if (config_.steering_motors == nullptr) {
    XLOG_ERROR("Steering motor group is not initialized");
    return;
  }
  // set steering motors angle
  std::vector<float> steering_angles;
  for (int i = 0; i < 4; ++i) {
    steering_angles.push_back(angles[i] / M_PI * 180.0f);
  }
  config_.steering_motors->SetPositions(steering_angles);
}

void SwerveDriveRobot::SetDrivingCommand(const std::array<float, 4>& speeds) {
  if (config_.driving_motors == nullptr) {
    XLOG_ERROR("Driving motor group is not initialized");
    return;
  }
  // set driving motors speed
  std::vector<float> driving_speeds;
  for (int i = 0; i < 4; ++i) {
    // convert speed m/s to rpm
    float rpm = speeds[i] / (2 * M_PI * config_.wheel_radius) * 60;
    if (config_.reverse_left_wheels && (i == 1 || i == 2)) {
      driving_speeds.push_back(-rpm);
    } else if (config_.reverse_right_wheels && (i == 0 || i == 3)) {
      driving_speeds.push_back(-rpm);
    } else {
      driving_speeds.push_back(rpm);
    }
  }
  config_.driving_motors->SetSpeeds(driving_speeds);
}
}  // namespace xmotion