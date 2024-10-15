/*
 * @file ddsm_210.cpp
 * @date 10/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/ddsm_210.hpp"

#include <array>
#include <sstream>

#include "async_port/async_serial.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
// reference:
// https://reveng.sourceforge.io/crc-catalogue/all.htm#crc.cat.crc-8-maxim-dow
int8_t CalculateChecksum(const std::array<uint8_t, 10>& data) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < data.size() - 1; ++i) {
    crc ^= data[i];
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

int8_t CalculateChecksum(const std::vector<uint8_t>& data) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < data.size() - 1; ++i) {
    crc ^= data[i];
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

void PrintBuffer(const std::array<uint8_t, 10>& buffer) {
  std::cout << "Buffer: ";
  for (int i = 0; i < buffer.size(); i++) {
    std::cout << std::hex << int(buffer[i]) << " ";
  }
  std::cout << std::endl;
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

void Ddsm210::SetAcceleration(uint8_t ms_per_rpm) {
  ms_per_rpm_ = ms_per_rpm;
  //  int16_t rpm_val = static_cast<int16_t>(rpm * 10);

  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x64;
  tx_buffer_[2] = 0;
  tx_buffer_[3] = 0;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = ms_per_rpm;
  tx_buffer_[7] = 0;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
}

void Ddsm210::SetSpeed(int32_t rpm) {
  if (rpm > max_rpm) rpm = max_rpm;
  if (rpm < min_rpm) rpm = min_rpm;

  int16_t rpm_val = static_cast<int16_t>(rpm * 10);

  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x64;
  tx_buffer_[2] = (rpm_val & 0xff00) >> 8;
  tx_buffer_[3] = rpm_val & 0x00ff;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = 0;
  tx_buffer_[7] = 0;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
  serial_->SendBytes(tx_buffer_.data(), tx_buffer_.size());

  RequestOdometryFeedback();
}

int32_t Ddsm210::GetSpeed() { return raw_feedback_.rpm; }

int32_t Ddsm210::GetEncoderCount() { return raw_feedback_.encoder_count; }

void Ddsm210::SetPosition(double position) {
  if (position > max_pos) position = max_pos;
  if (position < min_pos) position = min_pos;

  int16_t pos_val = static_cast<int16_t>(position / 360 * 32767);
  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x64;
  tx_buffer_[2] = (pos_val & 0xff00) >> 8;
  tx_buffer_[3] = pos_val & 0x00ff;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = 0;
  tx_buffer_[7] = 0;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
  serial_->SendBytes(tx_buffer_.data(), tx_buffer_.size());

  RequestOdometryFeedback();
}

double Ddsm210::GetPosition() {
  return raw_feedback_.position * 360.0f / 32767;
}

void Ddsm210::ApplyBrake(double brake) {
  (void)brake;
  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x64;
  tx_buffer_[2] = 0;
  tx_buffer_[3] = 0;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = 0;
  tx_buffer_[7] = 0xff;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
  serial_->SendBytes(tx_buffer_.data(), tx_buffer_.size());
}

void Ddsm210::ReleaseBrake() {
  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x64;
  tx_buffer_[2] = 0;
  tx_buffer_[3] = 0;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = 0;
  tx_buffer_[7] = 0;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
  serial_->SendBytes(tx_buffer_.data(), tx_buffer_.size());
}

bool Ddsm210::IsNormal() {
  return serial_->IsOpened() && (raw_feedback_.error_code == 0x00);
}

void Ddsm210::RequestOdometryFeedback() {
  tx_buffer_[0] = motor_id_;
  tx_buffer_[1] = 0x74;
  tx_buffer_[2] = 0;
  tx_buffer_[3] = 0;
  tx_buffer_[4] = 0;
  tx_buffer_[5] = 0;
  tx_buffer_[6] = 0;
  tx_buffer_[7] = 0;
  tx_buffer_[8] = 0;
  tx_buffer_[9] = CalculateChecksum(tx_buffer_);
  serial_->SendBytes(tx_buffer_.data(), tx_buffer_.size());
}

void Ddsm210::ProcessFeedback(uint8_t* data, const size_t bufsize, size_t len) {
  {
    std::stringstream ss;
    for (int i = 0; i < len; i++) {
      ss << std::hex << (int)(data[i]) << " ";
    }
    XLOG_INFO_STREAM("Raw feedback: " << ss.str());
  }

  std::vector<uint8_t> new_data(data, data + len);
  rx_buffer_.Write(new_data, new_data.size());

  // only try to decode when there are at least 10 bytes in the buffer
  while (rx_buffer_.GetOccupiedSize() >= 10) {
    std::vector<uint8_t> frame;
    rx_buffer_.Peek(frame, 10);

    // check if the frame is valid
    if (frame[0] == motor_id_ &&
        (frame[1] == 0x64 || frame[1] == 0x74 || frame[1] == 0x75 ||
         frame[1] == 0xa0) &&
        CalculateChecksum((frame)) == frame[9]) {
      rx_buffer_.Read(frame, 10);
      if (data[1] == 0x64) {
        XLOG_INFO("Type: 0x64");
        raw_feedback_.rpm =
            static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                                 static_cast<uint16_t>(data[3]));
        raw_feedback_.current =
            static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                                 static_cast<uint16_t>(data[5]));
        raw_feedback_.ms_per_rpm = data[6];
        raw_feedback_.temperature = static_cast<int8_t>(data[7]);
        //      raw_feedback_.error_code = data[8];
      } else if (data[1] == 0x74) {
        XLOG_INFO("Type: 0x74");
        raw_feedback_.encoder_count =
            static_cast<int32_t>((static_cast<uint32_t>(data[2]) << 24) |
                                 (static_cast<uint32_t>(data[3]) << 16) |
                                 (static_cast<uint32_t>(data[4]) << 8) |
                                 static_cast<uint32_t>(data[5]));
        raw_feedback_.position =
            static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                 static_cast<uint16_t>(data[7]));
        raw_feedback_.error_code = data[8];
      } else if (data[1] == 0x75) {
        XLOG_INFO("Type: 0x75");
        raw_feedback_.mode = data[2];
      } else if (data[1] == 0xa0) {
        XLOG_INFO("Type: 0xa0");
        mode_set_ack_received_ = true;
        raw_feedback_.mode = data[2];
      }
    }
    // incomplete/invalid frame, discard the first byte and try again
    else {
      rx_buffer_.Read(frame, 1);
    }
  }
}
}  // namespace xmotion