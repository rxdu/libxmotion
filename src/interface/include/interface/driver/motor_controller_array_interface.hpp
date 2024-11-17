/*
 * motor_controller_array_interface.hpp
 *
 * Created on 4/21/24 10:07 PM
 * Description: if more than one motors can be connected to the same bus/network
 * and controlled from a single interface, then we can treat them as an array
 * and use this interface to control them
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTOR_CONTROLLER_ARRAY_INTERFACE_HPP
#define XMOTION_MOTOR_CONTROLLER_ARRAY_INTERFACE_HPP

#include <cstdint>
#include <stdexcept>

namespace xmotion {
class MotorControllerArrayInterface {
 public:
  virtual ~MotorControllerArrayInterface() = default;

  // public interface
  virtual void RegisterMotor(uint8_t id) = 0;
  virtual void UnregisterMotor(uint8_t id) = 0;

  virtual void SetSpeed(uint8_t id, float rpm) {
    throw std::runtime_error("SetSpeed not implemented");
  };

  virtual void SetSpeeds(const std::vector<float>& speeds) {
    throw std::runtime_error("SetSpeeds not implemented");
  };

  virtual void SetPosition(uint8_t id, float position) {
    throw std::runtime_error("SetPosition not implemented");
  };

  virtual void SetPositions(const std::vector<float>& positions) {
    throw std::runtime_error("SetPositions not implemented");
  };

  virtual void SetTorque(uint8_t id, float torque) {
    throw std::runtime_error("SetTorque not implemented");
  };

  virtual void SetTorques(const std::vector<float>& torques) {
    throw std::runtime_error("SetTorques not implemented");
  };

  virtual void SetCurrent(uint8_t id, float current) {
    throw std::runtime_error("SetCurrent not implemented");
  };

  virtual void SetCurrents(const std::vector<float>& currents) {
    throw std::runtime_error("SetCurrents not implemented");
  };

  virtual float GetSpeed(uint8_t id) {
    throw std::runtime_error("GetSpeed not implemented");
    return 0;
  };

  virtual void GetSpeeds(std::vector<float>& speeds) {
    throw std::runtime_error("GetSpeeds not implemented");
  };

  virtual float GetPosition(uint8_t id) {
    throw std::runtime_error("GetPosition not implemented");
    return 0;
  };

  virtual void GetPositions(std::vector<float>& positions) {
    throw std::runtime_error("GetPositions not implemented");
  };

  virtual float GetTorque(uint8_t id) {
    throw std::runtime_error("GetTorque not implemented");
    return 0;
  };

  virtual void GetTorques(std::vector<float>& torques) {
    throw std::runtime_error("GetTorques not implemented");
  };

  virtual float GetCurrent(uint8_t id) {
    throw std::runtime_error("GetCurrent not implemented");
    return 0;
  };

  virtual void GetCurrents(std::vector<float>& currents) {
    throw std::runtime_error("GetCurrents not implemented");
  };

  virtual void ApplyBrake(uint8_t id, float brake) {
    throw std::runtime_error("ApplyBrake not implemented");
  };

  virtual void ReleaseBrake(uint8_t id) {
    throw std::runtime_error("ReleaseBrake not implemented");
  };

  virtual bool IsNormal(uint8_t id) {
    throw std::runtime_error("IsNormal not implemented");
    return true;
  };
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_CONTROLLER_ARRAY_INTERFACE_HPP
