/*
 * @file modbus_rtu_port.hpp
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
class ModbusRtuPort : public ModbusRtuInterface {
 public:
  ModbusRtuPort();

  // public methods
  // port management
  bool Open(const std::string& port_name, int baud_rate, Parity parity,
            DataBit data_bit, StopBit stop_bit) override;
  void Close() override;
  bool IsOpened() const override;

  bool SetResponseTimeout(int sec, int usec) override;

  // modbus read/write
  // function code 01
  int ReadCoils(uint8_t device_id, uint16_t addr, uint16_t quantity,
                uint8_t* data) override;
  // function code 05
  int WriteSingleCoil(uint8_t device_id, uint16_t addr, int status) override;
  // function code 15
  int WriteMultipleCoils(uint8_t device_id, uint16_t addr, uint16_t quantity,
                         const uint8_t* data) override;

  // function code 02
  int ReadDiscreteInputs(uint8_t device_id, uint16_t addr, uint16_t quantity,
                         uint8_t* data) override;

  // function code 04
  int ReadInputRegisters(uint8_t device_id, uint16_t addr, uint16_t quantity,
                         uint16_t* data) override;

  // function code 03
  int ReadHoldingRegisters(uint8_t device_id, uint16_t addr, uint16_t quantity,
                           uint16_t* data) override;

  // function code 06
  int WriteSingleRegister(uint8_t device_id, uint16_t addr,
                          uint16_t value) override;
  // function code 16
  int WriteMultipleRegisters(uint8_t device_id, uint16_t addr,
                             uint16_t quantity, const uint16_t* data) override;

 private:
  bool SelectDevice(uint8_t device_id) override;

  modbus_t* ctx_ = nullptr;
};
}  // namespace xmotion

#endif  // XMOTION_MODBUS_PORT_HPP
