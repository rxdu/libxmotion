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
  virtual void SetSpeed(int32_t rpm) = 0;
  virtual void SetPosition(double position) {};
  virtual void SetTorque(double torque) {};
  virtual void SetCurrent(double current) {};

  virtual int32_t GetSpeed() = 0;

  virtual double GetPosition() { return 0; };

  virtual double GetTorque() { return 0; };

  virtual double GetCurrent() { return 0; };

  virtual void ApplyBrake(double brake) {};
  virtual void ReleaseBrake() {};

  virtual bool IsNormal() { return true; };
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_
