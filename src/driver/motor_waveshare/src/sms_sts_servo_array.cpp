/*
 * @file sms_sts_servo_array.cpp
 * @date 11/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <cassert>
#include <algorithm>

#include "motor_waveshare/sms_sts_servo_array.hpp"

namespace xmotion {
SmsStsServoArray::SmsStsServoArray(const std::string& dev_name)
    : dev_name_(dev_name) {}

void SmsStsServoArray::SetDefaultPosition(float position) {
  default_position_ = position;
}

void SmsStsServoArray::RegisterMotor(uint8_t id) {
  ids_.push_back(id);
  positions_targets_[id] = default_position_;
}

void SmsStsServoArray::UnregisterMotor(uint8_t id) {
  ids_.erase(std::find(ids_.begin(), ids_.end(), id));
}

bool SmsStsServoArray::Connect() {
  servo_ = std::make_shared<SmsStsServo>(ids_);
  servo_->SetPositionOffset(position_offset_);
  return servo_->Connect(dev_name_);
}

void SmsStsServoArray::Disconnect() { servo_->Disconnect(); }

void SmsStsServoArray::SetPositionOffset(float offset) {
  position_offset_ = offset;
}

void SmsStsServoArray::SetPosition(uint8_t id, float position) {
  if (position == positions_targets_[id]) return;
  positions_targets_[id] = position;
  std::vector<float> positions;
  for (auto id : ids_) {
    positions.push_back(positions_targets_[id]);
  }
  servo_->SetPosition(positions);
}

void SmsStsServoArray::SetPositions(const std::vector<float>& positions) {
  assert(positions.size() == ids_.size());
  std::vector<float> cmds;
  for (int i = 0; i < ids_.size(); i++) {
    cmds.push_back(positions[i]);
  }
  servo_->SetPosition(cmds);
}

float SmsStsServoArray::GetPosition(uint8_t id) {
  auto states = servo_->GetStates();
  return states[id].position;
}

bool SmsStsServoArray::IsNormal(uint8_t id) {
  // TODO: implement this by checking the state of each motor
  //  auto state = servo_->GetStates();
  return true;
}
}  // namespace xmotion