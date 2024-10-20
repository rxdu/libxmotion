/*
 * @file sms_sts_servo_impl.cpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/sms_sts_servo.hpp"

#include "SCServo.h"

#include "logging/xlogger.hpp"

namespace xmotion {
class SmsStsServo::Impl {
 public:
  Impl(uint8_t id) : id_(id) {};
  ~Impl() = default;

  bool Connect(std::string dev_name) {
    if (!sm_st_.begin(1000000, dev_name.c_str())) {
      XLOG_ERROR("Failed to init sms/sts motor!");
      return false;
    }
    return true;
  }

  void Disconnect() { sm_st_.end(); }

  void SetSpeed(float rpm) {}

  float GetSpeed() { return 0; }

  void SetPosition(float position) {
    // map 0-360 to 0-4095
    s16 pos = position / 360 * 4095;
    XLOG_INFO_STREAM("Set motor pos: " << pos);
    sm_st_.WritePosEx(id_, pos, 2400, 50);
  }

  float GetPosition() { return 0.0; }

  bool IsNormal() { return true; }

  SmsStsServo::State GetState() {
    if (sm_st_.FeedBack(1) != -1) {
      state_.position = sm_st_.ReadPos(-1);  //-1表示缓冲区数据，以下相同
      state_.speed = sm_st_.ReadSpeed(-1);
      state_.load = sm_st_.ReadLoad(-1);
      state_.voltage = sm_st_.ReadVoltage(-1);
      state_.temperature = sm_st_.ReadTemper(-1);
      state_.is_moving = sm_st_.ReadMove(-1);
      state_.current = sm_st_.ReadCurrent(-1);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      XLOG_WARN("Failed to read motor state");
    }
    return state_;
  }

  bool SetMode(Mode mode, uint32_t timeout_ms) {
    if (mode == Mode::kSpeed) {
      sm_st_.WheelMode(id_);
    }
    return false;
  }

 private:
  uint8_t id_ = 0;
  SMS_STS sm_st_;
  State state_;
};
}  // namespace xmotion