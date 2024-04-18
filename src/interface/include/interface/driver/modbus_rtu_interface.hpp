/*
 * @file modbus_rtu_interface.hpp
 * @date 4/18/24
 * @brief
 * Reference:
 * [1] https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
 * [2] https://www.modbus.org/docs/Modbus_over_serial_line_V1.pdf
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MODBUS_RTU_INTERFACE_HPP
#define XMOTION_MODBUS_RTU_INTERFACE_HPP

#include <string>

namespace xmotion {
class ModbusRtuInterface {
 public:
  enum class Parity : char { kNone = 'N', kEven = 'E', kOdd = 'O' };
  enum class DataBit : int { kBit5 = 5, kBit6 = 6, kBit7 = 7, kBit8 = 8 };
  enum class StopBit : int { kBit1 = 1, kBit2 = 2 };

  virtual ~ModbusRtuInterface() = default;

  // port management
  virtual bool Open(const std::string& port_name, int baud_rate, Parity parity,
                    DataBit data_bit, StopBit stop_bit) = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  // modbus read/write
  // function code 01
  virtual bool ReadCoils(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                         uint8_t* data) = 0;
  // function code 05
  virtual bool WriteSingleCoil(uint8_t slave_id, uint16_t addr,
                               bool status) = 0;
  // function code 15
  virtual bool WriteMultipleCoils(uint8_t slave_id, uint16_t addr,
                                  uint16_t quantity, const uint8_t* data) = 0;

  // function code 02
  virtual bool ReadDiscreteInputs(uint8_t slave_id, uint16_t addr,
                                  uint16_t quantity, uint8_t* data) = 0;

  // function code 04
  virtual bool ReadInputRegisters(uint8_t slave_id, uint16_t addr,
                                  uint16_t quantity, uint16_t* data) = 0;

  // function code 03
  virtual bool ReadHoldingRegisters(uint8_t slave_id, uint16_t addr,
                                    uint16_t quantity, uint16_t* data) = 0;
  // function code 06
  virtual bool WriteSingleRegister(uint8_t slave_id, uint16_t addr,
                                   uint16_t value) = 0;
  // function code 16
  virtual bool WriteMultipleRegisters(uint8_t slave_id, uint16_t addr,
                                      uint16_t quantity,
                                      const uint16_t* data) = 0;
};
}  // namespace xmotion

#endif  // XMOTION_MODBUS_RTU_INTERFACE_HPP
