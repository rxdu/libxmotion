/*
 * motor_akelc.cpp
 *
 * Created on 4/20/24 11:44 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_akelc/motor_akelc.hpp"

namespace xmotion {
MotorAkelc::MotorAkelc(std::shared_ptr<MotorAkelcInterface> impl)
    : impl_(impl) {}

void MotorAkelc::SetSpeed(float rpm) { impl_->SetTargetRpm(rpm); }

float MotorAkelc::GetSpeed() { return impl_->GetActualRpm(); }

void MotorAkelc::ApplyBrake(float brake) { impl_->ApplyBrake(brake); }

void MotorAkelc::ReleaseBrake() { impl_->ReleaseBrake(); }

bool MotorAkelc::IsNormal() {
  if (!impl_->IsMotorBlocked() &&
      impl_->GetErrorCode() == MotorAkelcInterface::ErrorCode::kNoError) {
    return true;
  }
  return false;
}
}  // namespace xmotion