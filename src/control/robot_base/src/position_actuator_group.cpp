/*
 * @file position_actuator_group.cpp
 * @date 11/9/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/actuator/position_actuator_group.hpp"

namespace xmotion {
void PositionActuatorGroup::SetPosition(float position) {
  for (auto &actuator : actuators_) actuator->SetPosition(position);
}

float PositionActuatorGroup::GetPosition() {
  int64_t average_angle = 0;
  for (auto &actuator : actuators_) average_angle += actuator->GetPosition();
  return average_angle / actuators_.size();
}

bool PositionActuatorGroup::IsNormal() {
  for (auto &actuator : actuators_)
    if (!actuator->IsNormal()) return false;
  return true;
}
}  // namespace xmotion