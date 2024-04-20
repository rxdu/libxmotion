/*
 * @file motor_akelc_modbus.hpp
 * @date 4/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTOR_AKELC_MODBUS_HPP_
#define XMOTION_MOTOR_AKELC_MODBUS_HPP_

#include "motor_akelc/motor_akelc_interface.hpp"
#include "interface/driver/modbus_rtu_client.hpp"

namespace xmotion {
class MotorAkelcModbus final : public MotorAkelcInterface,
                               public ModbusRtuClient {
 public:
  MotorAkelcModbus(std::shared_ptr<ModbusRtuInterface> port, int device_id);
  ~MotorAkelcModbus() = default;

  // public interface
  bool IsReachable() override;
  std::string GetDeviceName() override;

  bool SetTargetSwitchingFreq(int16_t freq) override;
  int16_t GetActualSwitchingFreq() override;

  bool SetTargetRpm(int32_t rpm) override;
  int32_t GetActualRpm() override;

  bool ApplyBrake(float percentage) override;
  bool ReleaseBrake() override;

  double GetDriverCurrent() override;
  double GetDriverPwm() override;
  double GetDriverTemperature() override;
  double GetDriverInputVoltage() override;

  bool IsMotorBlocked() override;
  ErrorCode GetErrorCode() override;

 private:
  uint16_t buffer_[16];
};
}  // namespace xmotion

#endif  // XMOTION_MOTOR_AKELC_MODBUS_HPP_
