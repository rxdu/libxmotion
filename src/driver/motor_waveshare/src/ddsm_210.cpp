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
  return true;
}

Ddsm210::Mode Ddsm210::GetMode() const { return Ddsm210::Mode::kSpeed; }

bool Ddsm210::SetMotorId(uint8_t id, uint32_t timeout_ms) { return false; }

void Ddsm210::SetAcceleration(uint8_t ms_per_rpm) {}

void Ddsm210::SetSpeed(int32_t rpm) {
  Ddsm210Frame frame(motor_id_);
  frame.SetSpeed(rpm);
  auto buffer = frame.ToBuffer();
  //  PrintBuffer(buffer);
  serial_->SendBytes(buffer.data(), buffer.size());
}

int32_t Ddsm210::GetSpeed() { return raw_feedback_.rpm * 0.1f; }

int32_t Ddsm210::GetEncoderCount() { return raw_feedback_.encoder_count; }

void Ddsm210::SetPosition(double position) {
  Ddsm210Frame frame(motor_id_);
  frame.SetPosition(position);
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

double Ddsm210::GetPosition() {
  return raw_feedback_.position * 360.0f / 32767;
}

void Ddsm210::ApplyBrake(double brake) {
  Ddsm210Frame frame(motor_id_);
  frame.ApplyBrake();
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

void Ddsm210::ReleaseBrake() {
  Ddsm210Frame frame(motor_id_);
  frame.ReleaseBrake();
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

bool Ddsm210::IsNormal() {
  return serial_->IsOpened() && (raw_feedback_.error_code == 0x00);
}

void Ddsm210::RequestOdometryFeedback() {
  Ddsm210Frame frame(motor_id_);
  frame.RequestOdom();
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

void Ddsm210::ProcessFeedback(uint8_t* data, const size_t bufsize, size_t len) {
  //  {
  //    std::stringstream ss;
  //    for (int i = 0; i < len; i++) {
  //      ss << std::hex << (int)(data[i]) << " ";
  //    }
  //    XLOG_INFO_STREAM("Raw data: " << ss.str());
  //  }

  std::vector<uint8_t> new_data(data, data + len);
  rx_buffer_.Write(new_data, new_data.size());

  // only try to decode when there are at least 10 bytes in the buffer
  while (rx_buffer_.GetOccupiedSize() >= 10) {
    std::vector<uint8_t> frame(10);
    rx_buffer_.Peek(frame, 10);
    Ddsm210Frame ddsm_frame(frame);
    if (ddsm_frame.IsValid()) {
      //      XLOG_INFO("----------------------> Frame found");
      raw_feedback_ = ddsm_frame.GetRawFeedback();
      rx_buffer_.Read(frame, 10);
    } else {
      //      XLOG_INFO("xxxxxxxxxxxxxxxxxxx");
      // incomplete/invalid frame, discard the first byte and try again
      rx_buffer_.Read(frame, 1);
    }
  }
}
}  // namespace xmotion