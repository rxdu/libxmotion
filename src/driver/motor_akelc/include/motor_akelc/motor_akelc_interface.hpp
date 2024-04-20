/*
 * @file motor_akelc.hpp
 * @date 4/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTOR_AKELC_INTERFACE_HPP_
#define XMOTION_MOTOR_AKELC_INTERFACE_HPP_

#include <cstdint>
#include <string>

namespace xmotion {
class MotorAkelcInterface {
 public:
  enum class ErrorCode : int {
    kNoError = 0,
    kBlocked = 2,
    kUnableToReachTargetRpm = 4,
    kOverCurrentStop = 6,
    kOverHeatStop = 7,
    kOverVoltageStop = 8,
    kUnderVoltageStop = 9,
    kCircuitShortStop = 10,
    kBrakingCurrentException = 11,
    kUnknown = 20
  };

 public:
  virtual ~MotorAkelcInterface() = default;

  // public interface
  virtual std::string GetDeviceName() = 0;

  virtual bool SetTargetSwitchingFreq(int16_t freq) = 0;
  virtual int16_t GetActualSwitchingFreq() = 0;

  virtual bool SetTargetRpm(int32_t rpm) = 0;
  virtual int32_t GetActualRpm() = 0;

  virtual bool ApplyBrake(float percentage) = 0;
  virtual bool ReleaseBrake() = 0;

  virtual double GetDriverCurrent() = 0;
  virtual double GetDriverPwm() = 0;
  virtual double GetDriverTemperature() = 0;
  virtual double GetDriverInputVoltage() = 0;

  virtual bool IsMotorBlocked() = 0;
  virtual ErrorCode GetErrorCode() = 0;
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_AKELC_INTERFACE_HPP_
