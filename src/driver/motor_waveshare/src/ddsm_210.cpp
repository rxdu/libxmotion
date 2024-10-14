/*
 * @file ddsm_210.cpp
 * @date 10/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/ddsm_210.hpp"

#include <sstream>

#include "async_port/async_serial.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
static const uint8_t crc_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA,
    0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5,
    0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F,
    0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E, 0x4F, 0x1C, 0x2D,
    0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51,
    0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29,
    0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3,
    0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
    0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD,
    0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1, 0xF0, 0xA3, 0x92,
    0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68,
    0xFF, 0xCE, 0x9D, 0xAC};

int8_t CalculateChecksum(const std::array<uint8_t, 10>& data) {
  uint8_t crc = 0x00;  // Initial value
  for (int i = 0; i < 8; i++) {
    crc = crc_table[crc ^ data[i]];  // XOR with byte and lookup the table
  }
  return crc;
}
}  // namespace

Ddsm210::Ddsm210(uint8_t id) : motor_id_(id) {}

bool Ddsm210::Connect(std::string dev_name) {
  serial_ = std::make_shared<AsyncSerial>(dev_name, 115200);
  if (serial_->Open()) {
    serial_->SetReceiveCallback(
        std::bind(&Ddsm210::ProcessFeedback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
    return true;
  }
  return serial_->IsOpened();
}

void Ddsm210::Disconnect() { serial_->Close(); }

bool Ddsm210::SetMode(Ddsm210::Mode mode, uint32_t timeout_ms) {
  if (mode == Ddsm210::Mode::kSpeed) {
  } else if (mode == Ddsm210::Mode::kPosition) {
  } else {
  }
}

Ddsm210::Mode Ddsm210::GetMode() const { return Ddsm210::Mode::kSpeed; }

bool Ddsm210::SetMotorId(uint8_t id, uint32_t timeout_ms) { return false; }

void Ddsm210::SetAcceleration(uint8_t ms_per_rpm) { ms_per_rpm_ = ms_per_rpm; }

void Ddsm210::SetSpeed(int32_t rpm) {
  buffer_[0] = motor_id_;
  buffer_[1] = 0x64;
  int16_t rpm_val = 0x0000ffff & rpm;
  buffer_[2] = (rpm_val & 0xff00) >> 8;
  buffer_[3] = rpm_val & 0x00ff;
  buffer_[4] = 0;
  buffer_[5] = 0;
  buffer_[6] = ms_per_rpm_;
  buffer_[7] = 0;
  buffer_[8] = CalculateChecksum(buffer_);
  serial_->SendBytes(buffer_.data(), buffer_.size());

  RequestOdometryFeedback();
}

int32_t Ddsm210::GetSpeed() { return raw_feedback_.rpm; }

int32_t Ddsm210::GetEncoderCount() { return raw_feedback_.encoder_count; }

void Ddsm210::SetPosition(double position) {
  buffer_[0] = motor_id_;
  buffer_[1] = 0x64;
  int16_t pos_val = 0x0000ffff & static_cast<int16_t>(position / 360 * 32767);
  buffer_[2] = (pos_val & 0xff00) >> 8;
  buffer_[3] = pos_val & 0x00ff;
  buffer_[4] = 0;
  buffer_[5] = 0;
  buffer_[6] = ms_per_rpm_;
  buffer_[7] = 0;
  buffer_[8] = CalculateChecksum(buffer_);
  serial_->SendBytes(buffer_.data(), buffer_.size());

  RequestOdometryFeedback();
}

double Ddsm210::GetPosition() {
  return raw_feedback_.position * 360.0f / 32767;
}

void Ddsm210::ApplyBrake(double brake) {
  buffer_[0] = motor_id_;
  buffer_[1] = 0x64;
  buffer_[2] = 0;
  buffer_[3] = 0;
  buffer_[4] = 0;
  buffer_[5] = 0;
  buffer_[6] = ms_per_rpm_;
  buffer_[7] = 0xff;
  buffer_[8] = CalculateChecksum(buffer_);
  serial_->SendBytes(buffer_.data(), buffer_.size());
}

void Ddsm210::ReleaseBrake() {
  buffer_[0] = motor_id_;
  buffer_[1] = 0x64;
  buffer_[2] = 0;
  buffer_[3] = 0;
  buffer_[4] = 0;
  buffer_[5] = 0;
  buffer_[6] = ms_per_rpm_;
  buffer_[7] = 0;
  buffer_[8] = CalculateChecksum(buffer_);
  serial_->SendBytes(buffer_.data(), buffer_.size());
}

bool Ddsm210::IsNormal() {
  return serial_->IsOpened() && (raw_feedback_.error_code == 0x00);
}

void Ddsm210::RequestOdometryFeedback() {
  buffer_[0] = motor_id_;
  buffer_[1] = 0x74;
  buffer_[2] = 0;
  buffer_[3] = 0;
  buffer_[4] = 0;
  buffer_[5] = 0;
  buffer_[6] = 0;
  buffer_[7] = 0;
  buffer_[8] = CalculateChecksum(buffer_);
  serial_->SendBytes(buffer_.data(), buffer_.size());
}

void Ddsm210::ProcessFeedback(uint8_t* data, const size_t bufsize, size_t len) {
  std::stringstream ss;
  for (int i = 0; i < len; i++) {
    ss << std::hex << data[i] << " ";
  }
  XLOG_INFO_STREAM("Received feedback: " << ss.str());

  // only process data frame with correct id, length and checksum
  if (len == 10 && data[0] == motor_id_ &&
      CalculateChecksum(*reinterpret_cast<std::array<uint8_t, 10>*>(data)) ==
          data[9]) {
    if (data[0] == motor_id_ && data[1] == 0x64) {
      raw_feedback_.rpm =
          static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                               static_cast<uint16_t>(data[3]));
      raw_feedback_.current =
          static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                               static_cast<uint16_t>(data[5]));
      raw_feedback_.ms_per_rpm = data[6];
      raw_feedback_.temperature = static_cast<int8_t>(data[7]);
      //      raw_feedback_.error_code = data[8];
    } else if (data[0] == motor_id_ && data[1] == 0x74) {
      raw_feedback_.encoder_count =
          static_cast<int32_t>((static_cast<uint32_t>(data[2]) << 24) |
                               (static_cast<uint32_t>(data[3]) << 16) |
                               (static_cast<uint32_t>(data[4]) << 8) |
                               static_cast<uint32_t>(data[5]));
      raw_feedback_.position =
          static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                               static_cast<uint16_t>(data[7]));
      raw_feedback_.error_code = data[8];
    } else if (data[0] == motor_id_ && data[1] == 0x75) {
      raw_feedback_.mode = data[2];
    } else if (data[0] == motor_id_ && data[1] == 0xa0) {
      mode_set_ack_received_ = true;
      raw_feedback_.mode = data[2];
    }
  }
}
}  // namespace xmotion