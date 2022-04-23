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
#define TBOT_RPM_CMD_CAN_ID 0x102
#define TBOT_MOTION_CMD_CAN_ID 0x103

#define TBOT_ENCODER_RAW_CAN_ID 0x211
#define TBOT_ENCODER_FILTERED_CAN_ID 0x212

namespace robosw {
namespace {
int32_t ToInt32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  int32_t ret = 0;
  ret = static_cast<int32_t>(
      static_cast<uint32_t>(b0) << 24 | static_cast<uint32_t>(b1) << 16 |
          static_cast<uint32_t>(b2) << 8 | static_cast<uint32_t>(b3) << 0);
  return ret;
}
}  // namespace

Messenger::Messenger(RSTimePoint tp) : t0_(tp) {
  rpm_buffers_[DataBufferIndex::kRawRpmLeft] = swviz::DataBuffer();
  rpm_buffers_[DataBufferIndex::kRawRpmRight] = swviz::DataBuffer();
  rpm_buffers_[DataBufferIndex::kFilteredRpmLeft] = swviz::DataBuffer();
  rpm_buffers_[DataBufferIndex::kFilteredRpmRight] = swviz::DataBuffer();

  rpm_buffers_[DataBufferIndex::kRawRpmLeft].Resize(4096);
  rpm_buffers_[DataBufferIndex::kRawRpmRight].Resize(4096);
  rpm_buffers_[DataBufferIndex::kFilteredRpmLeft].Resize(4096);
  rpm_buffers_[DataBufferIndex::kFilteredRpmRight].Resize(4096);
}

bool Messenger::Start(std::string can) {
  if (can_ != nullptr) can_.reset();
  can_ = std::make_shared<AsyncCAN>(can);
  can_->SetReceiveCallback(
      std::bind(&Messenger::HandleCanFrame, this, std::placeholders::_1));
  return can_->Open();
}

void Messenger::Stop() { can_->Close(); }

bool Messenger::IsStarted() {
  if (can_ == nullptr) return false;
  return can_->IsOpened();
}

swviz::DataBuffer &Messenger::GetDataBuffer(DataBufferIndex idx) {
  assert(rpm_buffers_.find(idx) != rpm_buffers_.end());
  return rpm_buffers_[idx];
}

void Messenger::HandleCanFrame(can_frame *rx_frame) {
  float t = std::chrono::duration_cast<std::chrono::milliseconds>(
      RSClock::now() - t0_)
      .count() /
      1000.0f;

  switch (rx_frame->can_id) {
    case TBOT_ENCODER_RAW_CAN_ID: {
      int32_t left = ToInt32(rx_frame->data[0], rx_frame->data[1],
                             rx_frame->data[2], rx_frame->data[3]);
      int32_t right = ToInt32(rx_frame->data[4], rx_frame->data[5],
                              rx_frame->data[6], rx_frame->data[7]);
      rpm_buffers_[DataBufferIndex::kRawRpmLeft].AddPoint(t, left / 1.0f);
      rpm_buffers_[DataBufferIndex::kRawRpmRight].AddPoint(t, right / 1.0f);
      break;
    }
    case TBOT_ENCODER_FILTERED_CAN_ID: {
      int32_t left = ToInt32(rx_frame->data[0], rx_frame->data[1],
                             rx_frame->data[2], rx_frame->data[3]);
      int32_t right = ToInt32(rx_frame->data[4], rx_frame->data[5],
                              rx_frame->data[6], rx_frame->data[7]);
      rpm_buffers_[DataBufferIndex::kFilteredRpmLeft].AddPoint(t, left / 1.0f);
      rpm_buffers_[DataBufferIndex::kFilteredRpmRight].AddPoint(t,
                                                                right / 1.0f);
      break;
    }
  }
}

void Messenger::SendPwmCommand(float left, float right) {
  if (can_ == nullptr) return;
  struct can_frame frame;
  frame.can_id = TBOT_PWM_CMD_CAN_ID;
  frame.can_dlc = 2;
  frame.data[0] = static_cast<uint8_t>(left);
  frame.data[1] = static_cast<uint8_t>(right);
  can_->SendFrame(frame);
}

void Messenger::SendRpmCommand(int32_t left, int32_t right) {
  if (can_ == nullptr) return;

  struct can_frame frame;
  frame.can_id = TBOT_RPM_CMD_CAN_ID;
  frame.can_dlc = 8;
  frame.data[0] =
      static_cast<uint8_t>((static_cast<uint32_t>(left) & 0xff000000) >> 24);
  frame.data[1] =
      static_cast<uint8_t>((static_cast<uint32_t>(left) & 0x00ff0000) >> 16);
  frame.data[2] =
      static_cast<uint8_t>((static_cast<uint32_t>(left) & 0x0000ff00) >> 8);
  frame.data[3] =
      static_cast<uint8_t>((static_cast<uint32_t>(left) & 0x000000ff) >> 0);

  frame.data[4] =
      static_cast<uint8_t>((static_cast<uint32_t>(right) & 0xff000000) >> 24);
  frame.data[5] =
      static_cast<uint8_t>((static_cast<uint32_t>(right) & 0x00ff0000) >> 16);
  frame.data[6] =
      static_cast<uint8_t>((static_cast<uint32_t>(right) & 0x0000ff00) >> 8);
  frame.data[7] =
      static_cast<uint8_t>((static_cast<uint32_t>(right) & 0x000000ff) >> 0);

  can_->SendFrame(frame);
}

void Messenger::SendMotionCommand(float linear, float angular) {
  if (can_ == nullptr) return;

  struct can_frame frame;
  frame.can_id = TBOT_MOTION_CMD_CAN_ID;
  frame.can_dlc = 8;

  int32_t linear_cmd = linear * 100;
  int32_t angular_cmd = angular * 100;

  frame.data[0] =
      static_cast<uint8_t>((static_cast<uint32_t>(linear_cmd) & 0xff000000) >> 24);
  frame.data[1] =
      static_cast<uint8_t>((static_cast<uint32_t>(linear_cmd) & 0x00ff0000) >> 16);
  frame.data[2] =
      static_cast<uint8_t>((static_cast<uint32_t>(linear_cmd) & 0x0000ff00) >> 8);
  frame.data[3] =
      static_cast<uint8_t>((static_cast<uint32_t>(linear_cmd) & 0x000000ff) >> 0);

  frame.data[4] =
      static_cast<uint8_t>((static_cast<uint32_t>(angular_cmd) & 0xff000000) >> 24);
  frame.data[5] =
      static_cast<uint8_t>((static_cast<uint32_t>(angular_cmd) & 0x00ff0000) >> 16);
  frame.data[6] =
      static_cast<uint8_t>((static_cast<uint32_t>(angular_cmd) & 0x0000ff00) >> 8);
  frame.data[7] =
      static_cast<uint8_t>((static_cast<uint32_t>(angular_cmd) & 0x000000ff) >> 0);

  can_->SendFrame(frame);
}
}  // namespace robosw