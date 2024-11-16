/*
 * @file ws_sbot_base.cpp
 * @date 11/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/ws_sbot_base.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
WsSbotBase::WsSbotBase(const SbotConfig::BaseConfig& config)
    : config_(config) {}

bool WsSbotBase::Initialize() {
  steering_motor_ =
      std::make_shared<SmsStsServoArray>(config_.steering_motor_port);
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
  driving_motor_ = std::make_shared<Ddsm210Array>(config_.driving_motor_port);
  driving_motor_->RegisterMotor(1);
  driving_motor_->RegisterMotor(2);
  driving_motor_->RegisterMotor(3);
  driving_motor_->RegisterMotor(4);
  if (!driving_motor_->Connect()) {
    std::cout << "Failed to connect to driving motor" << std::endl;
    return false;
  }

  ModelConfig config;
  config.track_width = sbot_track_width;
  config.wheel_base = sbot_wheel_base;
  config.wheel_radius = sbot_wheel_radius;
  config.steering_motors = steering_motor_;
  config.driving_motors = driving_motor_;
  config.reverse_right_wheels = true;
  robot_ = std::make_unique<SwerveDriveRobot>(config);

  return true;
}

void WsSbotBase::SetSteeringCommand(const std::array<float, 4>& angles) {
  if (robot_ == nullptr) {
    std::cout << "Robot model is not initialized" << std::endl;
    return;
  }
  robot_->SetSteeringCommand(angles);
}

void WsSbotBase::SetDrivingCommand(const std::array<float, 4>& speeds) {
  if (robot_ == nullptr) {
    std::cout << "Robot model is not initialized" << std::endl;
    return;
  }
  robot_->SetDrivingCommand(speeds);
}
}  // namespace xmotion