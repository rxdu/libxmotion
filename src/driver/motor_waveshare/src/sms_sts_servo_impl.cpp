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

  Impl(const std::vector<uint8_t>& ids) : ids_(ids) {
    if (ids.size() == 1) {
      id_ = ids[0];
    }
  }

  ~Impl() = default;

  bool Connect(std::string dev_name) {
    if (!sm_st_.begin(1000000, dev_name.c_str())) {
      XLOG_ERROR("Failed to init sms/sts motor!");
      return false;
    }
    return true;
  }

  void Disconnect() { sm_st_.end(); }

  void SetSpeed(float step_per_sec) {
    if (step_per_sec > 2400) step_per_sec = 2400;
    if (step_per_sec < -2400) step_per_sec = -2400;
    sm_st_.WriteSpe(id_, step_per_sec, 50);
  }

  float GetSpeed() {
    auto speed = sm_st_.ReadSpeed(id_);
    return speed;
  }

  void SetPosition(float position) {
    // map 0-360 to 0-4095
    s16 pos = position / 360.0f * 4095;
    XLOG_INFO_STREAM("Set motor pos: " << pos);
    sm_st_.WritePosEx(id_, pos, 2400, 50);
  }

  float GetPosition() {
    auto pos = sm_st_.ReadPos(id_);
    return pos;
  }

  bool IsNormal() { return true; }

  SmsStsServo::State GetState() {
    if (sm_st_.FeedBack(id_) != -1) {
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

  bool SetMotorId(uint8_t id) {
    sm_st_.unLockEprom(id_);
    int ack = sm_st_.writeByte(id_, SMSBL_ID, id);
    sm_st_.LockEprom(id);
    return (ack == 1);
  }

  bool SetNeutralPosition() {
    int ack = sm_st_.CalibrationOfs(id_);
    return (ack == 1);
  }

  void SetPosition(std::vector<float> positions) {
    assert(positions.size() == ids_.size());
    std::vector<s16> pos_vec;
    std::vector<u16> speed_vec(ids_.size(), 2400);
    std::vector<u8> acc_vec(ids_.size(), 50);
    for (int i = 0; i < ids_.size(); i++) {
      pos_vec.push_back(positions[i] / 360 * 4095);
    }
    sm_st_.SyncWritePosEx(ids_.data(), ids_.size(), pos_vec.data(),
                          speed_vec.data(), acc_vec.data());
  }

  std::unordered_map<uint8_t, float> GetPositions() {
    std::unordered_map<uint8_t, float> positions;
    for (auto id : ids_) {
      auto pos = sm_st_.ReadPos(id);
      positions[id] = pos;
    }
    return positions;
  }

  std::unordered_map<uint8_t, State> GetStates() {
    std::unordered_map<uint8_t, State> states;
    for (auto id : ids_) {
      State state;
      if (sm_st_.FeedBack(id) != -1) {
        state.position = sm_st_.ReadPos(-1);  //-1表示缓冲区数据，以下相同
        state.speed = sm_st_.ReadSpeed(-1);
        state.load = sm_st_.ReadLoad(-1);
        state.voltage = sm_st_.ReadVoltage(-1);
        state.temperature = sm_st_.ReadTemper(-1);
        state.is_moving = sm_st_.ReadMove(-1);
        state.current = sm_st_.ReadCurrent(-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      states[id] = state;
    }
    return states;
  }

 private:
  uint8_t id_ = 0;
  std::vector<uint8_t> ids_;
  SMS_STS sm_st_;
  State state_;
};
}  // namespace xmotion