/*
 * differential_drive_robot.cpp
 *
 * Created on 4/21/24 10:38 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/differential_drive_robot.hpp"

#include <cmath>

#include "logging/xlogger.hpp"

namespace xmotion {
DifferentialDriveRobot::DifferentialDriveRobot(const Config& config)
    : config_(config) {}

void DifferentialDriveRobot::SetMotionCommand(double linear_vel,
                                              double angular_vel) {
  if (std::abs(linear_vel) < config_.linear_vel_deadband) linear_vel = 0.0;
  if (std::abs(angular_vel) < config_.angular_vel_deadband) angular_vel = 0.0;

  // calculate left and right wheel speed
  double left_wheel_speed =
      (linear_vel - angular_vel * config_.track_width / 2) /
      config_.wheel_radius;
  double right_wheel_speed =
      (linear_vel + angular_vel * config_.track_width / 2) /
      config_.wheel_radius;
  double left_rpm = left_wheel_speed * 60.0f / (2 * M_PI);
  double right_rpm = right_wheel_speed * 60.0f / (2 * M_PI);

  if (config_.left_actuator_group == nullptr ||
      config_.right_actuator_group == nullptr) {
    XLOG_ERROR("Actuator group is not initialized");
    return;
  }

  if (config_.reverse_left_wheel) left_rpm = -left_rpm;
  if (config_.reverse_right_wheel) right_rpm = -right_rpm;

  XLOG_DEBUG("Set left wheel speed: {} rpm, right wheel speed: {} rpm",
             left_rpm, right_rpm);

  config_.left_actuator_group->SetSpeed(left_rpm);
  config_.right_actuator_group->SetSpeed(right_rpm);
}

void DifferentialDriveRobot::GetMotionState(double& linear_vel,
                                             double& angular_vel) {
  // get left and right wheel speed
  double left_wheel_rpm = config_.left_actuator_group->GetSpeed();
  double right_wheel_rpm = config_.right_actuator_group->GetSpeed();
  if (config_.reverse_left_wheel) left_wheel_rpm = -left_wheel_rpm;
  if (config_.reverse_right_wheel) right_wheel_rpm = -right_wheel_rpm;

  double left_wheel_speed = left_wheel_rpm * 2 * M_PI / 60.0f;
  double right_wheel_speed = right_wheel_rpm * 2 * M_PI / 60.0f;
  linear_vel =
      (left_wheel_speed + right_wheel_speed) / 2 * config_.wheel_radius;
  angular_vel = (right_wheel_speed - left_wheel_speed) * config_.wheel_radius /
                config_.track_width;

  XLOG_DEBUG("Get motion status: linear_vel: {:.2f}, angular_vel: {:.2f}",
             linear_vel, angular_vel);
}
}  // namespace xmotion