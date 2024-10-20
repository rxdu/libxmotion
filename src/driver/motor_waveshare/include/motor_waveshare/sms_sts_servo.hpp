/*
 * @file sms_sts_servo.hpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SMS_STS_SERVO_HPP
#define XMOTION_SMS_STS_SERVO_HPP

#include <string>
#include <cstdint>
#include <memory>

#include "interface/driver/motor_controller_interface.hpp"

namespace xmotion {
class SmsStsServo : public MotorControllerInterface {
 public:
  enum class Mode { kSpeed = 0, kPosition, kUnknown = 0xff };

  struct State {
    bool is_moving;
    float position;
    float speed;
    float load;
    float voltage;
    float temperature;
    float current;
  };

 public:
  SmsStsServo(uint8_t id);
  ~SmsStsServo();

  // do not allow copy
  SmsStsServo(const SmsStsServo&) = delete;
  SmsStsServo& operator=(const SmsStsServo&) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  void SetSpeed(float step_per_sec) override;
  float GetSpeed() override;

  void SetPosition(float position) override;
  float GetPosition() override;

  bool IsNormal() override;

  State GetState() const;

  // the following functions may not be called during normal motor operation
  // in most cases, motor id and mode should be set beforehand
  bool SetMode(Mode mode, uint32_t timeout_ms = 100);

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
}  // namespace xmotion

#endif  // XMOTION_SMS_STS_SERVO_HPP