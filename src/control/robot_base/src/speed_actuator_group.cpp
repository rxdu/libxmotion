/*
 * speed_actuator_group.cpp
 *
 * Created on 4/21/24 10:53 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/speed_actuator_group.hpp"

namespace xmotion {
void SpeedActuatorGroup::SetSpeed(int32_t rpm) {
  for (auto &actuator : actuators_) actuator->SetSpeed(rpm);
}

int32_t SpeedActuatorGroup::GetSpeed() {
  int64_t average_speed = 0;
  for (auto &actuator : actuators_) average_speed += actuator->GetSpeed();
  return average_speed / actuators_.size();
}

void SpeedActuatorGroup::ApplyBrake(double brake) {
  for (auto &actuator : actuators_) actuator->ApplyBrake(brake);
}

void SpeedActuatorGroup::ReleaseBrake() {
  for (auto &actuator : actuators_) actuator->ReleaseBrake();
}

bool SpeedActuatorGroup::IsNormal() {
  for (auto &actuator : actuators_)
    if (!actuator->IsNormal()) return false;
  return true;
}
}  // namespace xmotion