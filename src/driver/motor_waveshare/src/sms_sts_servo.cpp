/*
 * @file sms_sts_servo.cpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/sms_sts_servo.hpp"

#include "sms_sts_servo_impl.cpp"

namespace xmotion {
SmsStsServo::SmsStsServo(uint8_t id) : pimpl_(std::make_unique<Impl>(id)) {}

SmsStsServo::~SmsStsServo() = default;

bool SmsStsServo::Connect(std::string dev_name) {
  return pimpl_->Connect(dev_name);
}

void SmsStsServo::Disconnect() { pimpl_->Disconnect(); }

void SmsStsServo::SetSpeed(float step_per_sec) {
  pimpl_->SetSpeed(step_per_sec);
}

float SmsStsServo::GetSpeed() { return pimpl_->GetSpeed(); }

void SmsStsServo::SetPosition(float position) { pimpl_->SetPosition(position); }

float SmsStsServo::GetPosition() { return pimpl_->GetPosition(); }

bool SmsStsServo::IsNormal() { return pimpl_->IsNormal(); }

SmsStsServo::State SmsStsServo::GetState() const { return pimpl_->GetState(); }

bool SmsStsServo::SetMode(SmsStsServo::Mode mode, uint32_t timeout_ms) {
  return pimpl_->SetMode(mode, timeout_ms);
}
}  // namespace xmotion