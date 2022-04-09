/* 
 * messenger.cpp
 *
 * Created on 4/4/22 9:52 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/messenger.hpp"

#include <cassert>
#include <iostream>

#define TBOT_PWM_CMD_CAN_ID 0x101
#define TBOT_MOTOR_CMD_CAN_ID 0x102
#define TBOT_MOTION_CMD_CAN_ID 0x103

#define TBOT_ENCODER_RAW_CAN_ID 0x211

namespace robosw {
namespace {
int32_t ToInt32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  int32_t ret = 0;
  ret = static_cast<int32_t>(static_cast<uint32_t>(b0) << 24 |
      static_cast<uint32_t>(b1) << 16 |
      static_cast<uint32_t>(b2) << 8 |
      static_cast<uint32_t>(b3) << 0);
  return ret;
}
}

Messenger::Messenger(RSTimePoint tp) : t0_(tp) {
  rpm_buffers_[RpmIndex::kLeft] = swviz::DataBuffer();
  rpm_buffers_[RpmIndex::kRight] = swviz::DataBuffer();

  rpm_buffers_[RpmIndex::kLeft].Resize(4096);
  rpm_buffers_[RpmIndex::kRight].Resize(4096);
}

bool Messenger::Start(std::string can) {
  if (can_ != nullptr) can_.reset();
  can_ = std::make_shared<AsyncCAN>(can);
  can_->SetReceiveCallback(std::bind(&Messenger::HandleCanFrame, this, std::placeholders::_1));
  return can_->Open();
}

void Messenger::Stop() {
  can_->Close();
}

bool Messenger::IsStarted() {
  if (can_ == nullptr) return false;
  return can_->IsOpened();
}

void Messenger::SendPwmCommand(float left, float right) {
  if (can_ == nullptr) return;
  struct can_frame frame;
  frame.can_id = 0x101;
  frame.can_dlc = 2;
  frame.data[0] = static_cast<uint8_t>(left);
  frame.data[1] = static_cast<uint8_t>(right);
  can_->SendFrame(frame);
}

swviz::DataBuffer &Messenger::GetRpmBuffer(RpmIndex idx) {
  assert(rpm_buffers_.find(idx) != rpm_buffers_.end());
  return rpm_buffers_[idx];
}

void Messenger::HandleCanFrame(can_frame *rx_frame) {
  float t = std::chrono::duration_cast<std::chrono::milliseconds>(
      RSClock::now() - t0_).count() / 1000.0f;

  switch (rx_frame->can_id) {
    case TBOT_ENCODER_RAW_CAN_ID: {
      int32_t left = ToInt32(rx_frame->data[0], rx_frame->data[1],
                             rx_frame->data[2], rx_frame->data[3]);
      int32_t right = ToInt32(rx_frame->data[4], rx_frame->data[5],
                              rx_frame->data[6], rx_frame->data[7]);
      rpm_buffers_[RpmIndex::kLeft].AddPoint(t, left / 1.0f);
      rpm_buffers_[RpmIndex::kRight].AddPoint(t, right / 1.0f);
      break;
    }
  }
}
}