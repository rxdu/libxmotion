/*
 * @file motor_akelc.hpp
 * @date 4/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_
#define XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_

#include "interface/driver/motor_controller_interface.hpp"
#include "motor_akelc/motor_akelc_interface.hpp"

#ifdef AKELC_WITH_MODBUS
#include "motor_akelc/motor_akelc_modbus.hpp"
#endif

namespace xmotion {
class MotorAkelc final : public MotorControllerInterface {
 public:
  MotorAkelc(std::shared_ptr<MotorAkelcInterface> impl);

  // public interface
  void SetSpeed(int32_t rpm) override;
  int32_t GetSpeed() override;

  void ApplyBrake(double brake) override;
  void ReleaseBrake() override;

  bool IsNormal() override;

 private:
  std::shared_ptr<MotorAkelcInterface> impl_;
};
}  // namespace xmotion

#endif  // XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_
