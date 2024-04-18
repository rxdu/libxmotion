/*
 * @file modbus_rtu.hpp
 * @date 4/18/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MODBUS_PORT_HPP
#define XMOTION_MODBUS_PORT_HPP

#include "interface/driver/modbus_rtu_interface.hpp"

namespace xmotion {
class ModbusRtu {
 public:
  enum class Parity : char { kNone = 'N', kEven = 'E', kOdd = 'O' };
  enum class DataBit : int { kBit5 = 5, kBit6 = 6, kBit7 = 7, kBit8 = 8 };
  enum class StopBit : int { kBit1 = 1, kBit2 = 2 };

 public:
  ModbusRtu();
};
}  // namespace xmotion

#endif  // XMOTION_MODBUS_PORT_HPP
