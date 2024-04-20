/*
 * @file modbus_rtu.hpp
 * @date 4/18/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MODBUS_PORT_HPP
#define XMOTION_MODBUS_PORT_HPP

#include <modbus.h>

#include "interface/driver/modbus_rtu_interface.hpp"

namespace xmotion {
class ModbusRtu : public ModbusRtuInterface {
 public:
  ModbusRtu();

  // public methods
  // port management
  bool Open(const std::string& port_name, int baud_rate, Parity parity,
            DataBit data_bit, StopBit stop_bit) override;
  void Close() override;
  bool IsOpened() const override;

  // modbus read/write
  // function code 01
  bool ReadCoils(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                 uint8_t* data) override;
  // function code 05
  bool WriteSingleCoil(uint8_t slave_id, uint16_t addr, bool status) override;
  // function code 15
  bool WriteMultipleCoils(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                          const uint8_t* data) override;

  // function code 02
  bool ReadDiscreteInputs(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                          uint8_t* data) override;

  // function code 04
  bool ReadInputRegisters(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                          uint16_t* data) override;

  // function code 03
  bool ReadHoldingRegisters(uint8_t slave_id, uint16_t addr, uint16_t quantity,
                            uint16_t* data) override;

  // function code 06
  bool WriteSingleRegister(uint8_t slave_id, uint16_t addr,
                           uint16_t value) override;
  // function code 16
  bool WriteMultipleRegisters(uint8_t slave_id, uint16_t addr,
                              uint16_t quantity, const uint16_t* data) override;

 private:
  bool SetResponseTimeout(int sec, int usec);

  modbus_t* ctx_ = nullptr;
};
}  // namespace xmotion

#endif  // XMOTION_MODBUS_PORT_HPP
