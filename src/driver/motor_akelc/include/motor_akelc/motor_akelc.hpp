/*
 * @file motor_akelc.hpp
 * @date 4/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_
#define XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_

#include "motor_akelc/motor_akelc_interface.hpp"

#ifdef AKELC_WITH_MODBUS
#include "motor_akelc/motor_akelc_modbus.hpp"
#endif

namespace xmotion {
class MotorAkelc {
 public:
  MotorAkelc() = default;

  // public interface

};
}  // namespace xmotion

#endif  // XMOTION_SRC_DRIVER_MODBUS_RTU_INCLUDE_MODBUS_RTU_MOTOR_AKELC_HPP_
