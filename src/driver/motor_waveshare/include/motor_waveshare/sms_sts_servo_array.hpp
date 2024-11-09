/*
 * @file sms_sts_servo_array.hpp
 * @date 11/10/24
 * @brief
 *
 * Limitation:
 * Registration/un-registration of motors is only allowed before connecting.
 * Once connected, new motor registration/un-registration is not allowed.
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SMS_STS_SERVO_ARRAY_HPP
#define XMOTION_SMS_STS_SERVO_ARRAY_HPP

#include <string>
#include <cstdint>
#include <memory>
#include <vector>
#include <unordered_map>

#include "interface/driver/motor_controller_array_interface.hpp"
#include "motor_waveshare/sms_sts_servo.hpp"

namespace xmotion {
class SmsStsServoArray : public MotorControllerArrayInterface {
 public:
  SmsStsServoArray() = default;
  ~SmsStsServoArray() = default;

  // do not allow copy
  SmsStsServoArray(const SmsStsServoArray&) = delete;
  SmsStsServoArray& operator=(const SmsStsServoArray&) = delete;

  // public methods
  void SetDefaultPosition(float position);
  void RegisterMotor(uint8_t id) override;
  void UnregisterMotor(uint8_t id) override;

  bool Connect(std::string dev_name);
  void Disconnect();

  void SetPositionOffset(float offset);
  void SetPosition(uint8_t id, float position) override;
  void SetPositions(std::unordered_map<uint8_t, float> positions) override;
  float GetPosition(uint8_t id) override;

  bool IsNormal(uint8_t id) override;

 private:
  std::vector<uint8_t> ids_;
  std::shared_ptr<SmsStsServo> servo_;
  float default_position_ = 0;
  float position_offset_ = 0;
  std::unordered_map<uint8_t, float> positions_targets_;
};
}  // namespace xmotion

#endif  // XMOTION_SMS_STS_SERVO_ARRAY_HPP