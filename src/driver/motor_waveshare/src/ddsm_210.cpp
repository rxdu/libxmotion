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

Ddsm210::~Ddsm210() {
  SetSpeed(0);
}

Ddsm210::Ddsm210(uint8_t id, std::shared_ptr<SerialInterface> serial)
    : motor_id_(id), serial_(serial) {}

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
  if (mode != Ddsm210::Mode::kOpenLoop && mode != Ddsm210::Mode::kSpeed &&
      mode != Ddsm210::Mode::kPosition) {
    XLOG_WARN("Invalid mode");
    return false;
  }

  Ddsm210Frame frame(motor_id_);
  frame.SetMode(mode);
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
  for (int i = 0; i < 10; ++i) {
    RequestModeFeedback();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(timeout_ms / 10.0f)));
    if (GetMode() == mode) {
      return true;
    }
  }

  return true;
}

Ddsm210::Mode Ddsm210::GetMode() const {
  return static_cast<Ddsm210::Mode>(raw_feedback_.mode_request_feedback.mode);
}

bool Ddsm210::SetMotorId(uint8_t id, uint32_t timeout_ms) {
  if (id == 0xaa) {
    XLOG_WARN("0xaa is reserved for broadcast");
    return false;
  }
  id_set_ack_received_ = false;
  motor_id_ = id;
  for (int i = 0; i < 5; ++i) {
    Ddsm210Frame frame(motor_id_);
    frame.SetId(id);
    auto buffer = frame.ToBuffer();
    serial_->SendBytes(buffer.data(), buffer.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // wait for ack with the new id to be received
  for (int i = 0; i < 10; ++i) {
    if (id_set_ack_received_) return true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(timeout_ms / 10.0f)));
  }
  return false;
}

void Ddsm210::SetSpeed(float rpm) {
  Ddsm210Frame frame(motor_id_);
  frame.SetSpeed(rpm);
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

float Ddsm210::GetSpeed() { return raw_feedback_.speed_feedback.rpm * 0.1f; }

int32_t Ddsm210::GetEncoderCount() const {
  return raw_feedback_.odom_feedback.encoder_count;
}

void Ddsm210::SetPosition(float position) {
  Ddsm210Frame frame(motor_id_);
  frame.SetPosition(position);
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

float Ddsm210::GetPosition() {
  return raw_feedback_.odom_feedback.position * 360.0f / 32767;
}

void Ddsm210::ApplyBrake(float brake) {
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
  return serial_->IsOpened() &&
         (raw_feedback_.odom_feedback.error_code == 0x00);
}

void Ddsm210::RequestOdometryFeedback() {
  Ddsm210Frame frame(motor_id_);
  frame.RequestOdom();
  auto buffer = frame.ToBuffer();
  serial_->SendBytes(buffer.data(), buffer.size());
}

void Ddsm210::RequestModeFeedback() {
  Ddsm210Frame frame(motor_id_);
  frame.RequestMode();
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
    Ddsm210Frame ddsm_frame(motor_id_, frame);
    if (ddsm_frame.IsValid()) {
      //      XLOG_INFO("----------------------> Frame found");
      auto new_raw_feedback = ddsm_frame.GetRawFeedback();
      switch (ddsm_frame.GetType()) {
        case Ddsm210Frame::Type::kSpeedFeedback: {
          raw_feedback_.speed_feedback = new_raw_feedback.speed_feedback;
          // note: motor id feedback share the same 0x64 frame id
          id_set_ack_received_ = true;
          break;
        }
        case Ddsm210Frame::Type::kOdomFeedback: {
          raw_feedback_.odom_feedback = new_raw_feedback.odom_feedback;
          break;
        }
        case Ddsm210Frame::Type::kModeSwitchFeedback: {
          raw_feedback_.mode_switch_feedback =
              new_raw_feedback.mode_switch_feedback;
          break;
        }
        case Ddsm210Frame::Type::kModeRequestFeedback: {
          raw_feedback_.mode_request_feedback =
              new_raw_feedback.mode_request_feedback;
        }
      }
      rx_buffer_.Read(frame, 10);
    } else {
      // incomplete/invalid frame, discard the first byte and try again
      rx_buffer_.Read(frame, 1);
    }
  }
}
}  // namespace xmotion