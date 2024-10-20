/*
 * motor_controller_interface.hpp
 *
 * Created on 4/21/24 10:07 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_
#define XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_

#include <cstdint>

namespace xmotion {
class MotorControllerInterface {
 public:
  virtual ~MotorControllerInterface() = default;

  // public interface
  virtual void SetSpeed(float rpm) = 0;
  virtual void SetPosition(float position) {};
  virtual void SetTorque(float torque) {};
  virtual void SetCurrent(float current) {};

  virtual float GetSpeed() = 0;

  virtual float GetPosition() { return 0; };

  virtual float GetTorque() { return 0; };

  virtual float GetCurrent() { return 0; };

  virtual void ApplyBrake(float brake) {};
  virtual void ReleaseBrake() {};

  virtual bool IsNormal() { return true; };
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_
