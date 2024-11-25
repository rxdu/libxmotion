/*
 * motor_controller_interface.hpp
 *
 * Created on 4/21/24 10:07 PM
 * Description: Not all functions are required for all types of motors. if a
 * motor does not support a certain function, it should throw a runtime error.
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_
#define XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_

#include <cstdint>
#include <stdexcept>

namespace xmotion {
class MotorControllerInterface {
 public:
  virtual ~MotorControllerInterface() = default;

  // public interface
  virtual void SetSpeed(float rpm) = 0;

  virtual void SetPosition(float position) {
    throw std::runtime_error("SetPosition not implemented");
  };

  virtual void SetTorque(float torque) {
    throw std::runtime_error("SetTorque not implemented");
  };

  virtual void SetCurrent(float current) {
    throw std::runtime_error("SetCurrent not implemented");
  };

  virtual float GetSpeed() = 0;

  virtual float GetPosition() {
    throw std::runtime_error("GetPosition not implemented");
    return 0;
  };

  virtual float GetTorque() {
    throw std::runtime_error("GetTorque not implemented");
    return 0;
  };

  virtual float GetCurrent() {
    throw std::runtime_error("GetCurrent not implemented");
    return 0;
  };

  virtual void ApplyBrake(float brake) {
    throw std::runtime_error("ApplyBrake not implemented");
  };

  virtual void ReleaseBrake() {
    throw std::runtime_error("ReleaseBrake not implemented");
  };

  virtual bool IsNormal() {
    throw std::runtime_error("IsNormal not implemented");
    return true;
  };
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_CONTROLLER_INTERFACE_HPP_
