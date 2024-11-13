/*
 * @file ws_sbot_base.cpp
 * @date 11/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/ws_sbot_base.hpp"

namespace xmotion {
bool WsSbotBase::Initialize() {
  steering_motor_ = std::make_shared<SmsStsServoArray>("/dev/ttyUSB0");
  steering_motor_->SetPositionOffset(180);
  steering_motor_->SetDefaultPosition(0);
  steering_motor_->RegisterMotor(1);
  steering_motor_->RegisterMotor(2);
  steering_motor_->RegisterMotor(3);
  steering_motor_->RegisterMotor(4);
  if (!steering_motor_->Connect()) {
    std::cout << "Failed to connect to steering motor" << std::endl;
    return false;
  }

  // set up driving motors
  driving_motor_ = std::make_shared<Ddsm210Array>("/dev/ttyUSB1");
  driving_motor_->RegisterMotor(1);
  driving_motor_->RegisterMotor(2);
  driving_motor_->RegisterMotor(3);
  driving_motor_->RegisterMotor(4);
  if (!driving_motor_->Connect()) {
    std::cout << "Failed to connect to driving motor" << std::endl;
    return false;
  }

  SwerveDriveRobot::Config config;
  config.track_width = sbot_track_width;
  config.wheel_base = sbot_wheel_base;
  config.wheel_radius = sbot_wheel_radius;
  config.steering_motors = steering_motor_;
  config.driving_motors = driving_motor_;
  config.reverse_right_wheels = true;
  robot_ = std::make_unique<SwerveDriveRobot>(config);

  return true;
}
}  // namespace xmotion