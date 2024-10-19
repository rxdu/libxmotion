/*
 * @file ddsm_210_frame.cpp
 * @date 10/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/details/ddsm_210_frame.hpp"

#include <cstddef>

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
// reference:
// https://reveng.sourceforge.io/crc-catalogue/all.htm#crc.cat.crc-8-maxim-dow
int8_t CalculateChecksum(const std::array<uint8_t, 10>& frame_buffer) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < frame_buffer.size() - 1; ++i) {
    crc ^= frame_buffer[i];
    for (size_t j = 0; j < 8; ++j) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8c;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

int8_t CalculateChecksum(const std::vector<uint8_t>& frame_buffer) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < frame_buffer.size() - 1; ++i) {
    crc ^= frame_buffer[i];
    for (size_t j = 0; j < 8; ++j) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8c;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
}  // namespace

///////////////////////////////////////////////////////////////////////////////

Ddsm210Frame::Ddsm210Frame(uint8_t id) : motor_id_(id) {}

Ddsm210Frame::Ddsm210Frame(const std::vector<uint8_t>& frame_buffer) {
  valid_ = false;
  if (CalculateChecksum(frame_buffer) != frame_buffer[9]) return;

  // motor id matches and checksum is correct
  switch (frame_buffer[1]) {
    case 0x64: {
      //      XLOG_INFO("Type: 0x64");
      raw_feedback_.rpm =
          static_cast<int16_t>((static_cast<uint16_t>(frame_buffer[2]) << 8) |
                               static_cast<uint16_t>(frame_buffer[3]));
      raw_feedback_.current =
          static_cast<int16_t>((static_cast<uint16_t>(frame_buffer[4]) << 8) |
                               static_cast<uint16_t>(frame_buffer[5]));
      raw_feedback_.ms_per_rpm = frame_buffer[6];
      raw_feedback_.temperature = static_cast<int8_t>(frame_buffer[7]);
      valid_ = true;
      break;
    }
    case 0x74: {
      //      XLOG_INFO("Type: 0x74");
      //        raw_feedback_.encoder_count =
      //            static_cast<int32_t>((static_cast<uint32_t>(frame_buffer[2])
      //            << 24) |
      //                                 (static_cast<uint32_t>(frame_buffer[3])
      //                                 << 16) |
      //                                 (static_cast<uint32_t>(frame_buffer[4])
      //                                 << 8) |
      //                                 static_cast<uint32_t>(frame_buffer[5]));
      //        raw_feedback_.position =
      //            static_cast<int16_t>((static_cast<uint16_t>(frame_buffer[6])
      //            << 8)
      //            |
      //                                 static_cast<uint16_t>(frame_buffer[7]));
      //        raw_feedback_.error_code = frame_buffer[8];
      valid_ = true;
      break;
    }
    case 0x75: {
      //      XLOG_INFO("Type: 0x75");
      //        raw_feedback_.mode = frame_buffer[2];
      valid_ = true;
      break;
    }
    case 0xa0: {
      //      XLOG_INFO("Type: 0xa0");
      //        mode_set_ack_received_ = true;
      //        raw_feedback_.mode = frame_buffer[2];
      valid_ = true;
      break;
    }
    default: {
      break;
    }
  }
}

bool Ddsm210Frame::IsValid() const { return valid_; }

Ddsm210Frame::Type Ddsm210Frame::GetType() const { return type_; }

Ddsm210Frame::RawFeedback Ddsm210Frame::GetRawFeedback() const {
  return raw_feedback_;
}

void Ddsm210Frame::SetSpeed(int16_t rpm) {
  if (rpm > max_rpm) rpm = max_rpm;
  if (rpm < min_rpm) rpm = min_rpm;

  int16_t rpm_val = static_cast<int16_t>(rpm * 10);

  frame_buffer_[0] = motor_id_;
  frame_buffer_[1] = 0x64;
  frame_buffer_[2] = (rpm_val & 0xff00) >> 8;
  frame_buffer_[3] = rpm_val & 0x00ff;
  frame_buffer_[4] = 0;
  frame_buffer_[5] = 0;
  frame_buffer_[6] = 0;
  frame_buffer_[7] = 0;
  frame_buffer_[8] = 0;
  frame_buffer_[9] = CalculateChecksum(frame_buffer_);

  valid_ = true;
}

void Ddsm210Frame::SetPosition(int16_t position) {
  if (position > max_pos) position = max_pos;
  if (position < min_pos) position = min_pos;

  int16_t pos_val = static_cast<int16_t>(position / 360 * 32767);
  frame_buffer_[0] = motor_id_;
  frame_buffer_[1] = 0x64;
  frame_buffer_[2] = (pos_val & 0xff00) >> 8;
  frame_buffer_[3] = pos_val & 0x00ff;
  frame_buffer_[4] = 0;
  frame_buffer_[5] = 0;
  frame_buffer_[6] = 0;
  frame_buffer_[7] = 0;
  frame_buffer_[8] = 0;
  frame_buffer_[9] = CalculateChecksum(frame_buffer_);

  valid_ = true;
}

void Ddsm210Frame::ApplyBrake() {
  frame_buffer_[0] = motor_id_;
  frame_buffer_[1] = 0x64;
  frame_buffer_[2] = 0;
  frame_buffer_[3] = 0;
  frame_buffer_[4] = 0;
  frame_buffer_[5] = 0;
  frame_buffer_[6] = 0;
  frame_buffer_[7] = 0xff;
  frame_buffer_[8] = 0;
  frame_buffer_[9] = CalculateChecksum(frame_buffer_);

  valid_ = true;
}

void Ddsm210Frame::ReleaseBrake() {
  frame_buffer_[0] = motor_id_;
  frame_buffer_[1] = 0x64;
  frame_buffer_[2] = 0;
  frame_buffer_[3] = 0;
  frame_buffer_[4] = 0;
  frame_buffer_[5] = 0;
  frame_buffer_[6] = 0;
  frame_buffer_[7] = 0;
  frame_buffer_[8] = 0;
  frame_buffer_[9] = CalculateChecksum(frame_buffer_);

  valid_ = true;
}

void Ddsm210Frame::SetId(uint8_t id) {}

void Ddsm210Frame::SetMode(Ddsm210Frame::Mode mode) {}

void Ddsm210Frame::RequestOdom() {
  frame_buffer_[0] = motor_id_;
  frame_buffer_[1] = 0x74;
  frame_buffer_[2] = 0;
  frame_buffer_[3] = 0;
  frame_buffer_[4] = 0;
  frame_buffer_[5] = 0;
  frame_buffer_[6] = 0;
  frame_buffer_[7] = 0;
  frame_buffer_[8] = 0;
  frame_buffer_[9] = CalculateChecksum(frame_buffer_);

  valid_ = true;
}

std::array<uint8_t, 10> Ddsm210Frame::ToBuffer() { return frame_buffer_; }
}  // namespace xmotion