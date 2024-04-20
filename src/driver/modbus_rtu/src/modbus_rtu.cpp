/*
 * modbus_port.cpp
 *
 * Created on 4/18/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "modbus_rtu/modbus_rtu.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
ModbusRtu::ModbusRtu() {}

bool ModbusRtu::Open(const std::string &port_name, int baud_rate,
                     ModbusRtuInterface::Parity parity,
                     ModbusRtuInterface::DataBit data_bit,
                     ModbusRtuInterface::StopBit stop_bit) {
  if (ctx_ == nullptr) {
    ctx_ =
        modbus_new_rtu(port_name.c_str(), baud_rate, static_cast<char>(parity),
                       static_cast<int>(data_bit), static_cast<int>(stop_bit));
    if (ctx_ == nullptr) {
      XLOG_ERROR("Unable to create the libmodbus context");
      return false;
    }
  }

  // connect to port
  if (modbus_connect(ctx_) == -1) {
    XLOG_ERROR("Connection failed: {}", modbus_strerror(errno));
    return false;
  }

  // by default set 500ms timeout
  if (!SetResponseTimeout(0, 500000)) {
    XLOG_ERROR("Failed to set response timeout to default 500ms");
    return false;
  }
  return true;
}

void ModbusRtu::Close() {
  if (ctx_ != nullptr) {
    modbus_close(ctx_);
    modbus_free(ctx_);
  }
}

bool ModbusRtu::IsOpened() const { return (ctx_ != nullptr); }

bool ModbusRtu::SetResponseTimeout(int sec, int usec) {
  if (ctx_ == nullptr) {
    XLOG_ERROR("Modbus context not initialized");
    return false;
  }
#if (LIBMODBUS_VERSION_MAJOR == 3 && LIBMODBUS_VERSION_MINOR >= 1) || \
    (LIBMODBUS_VERSION_MAJOR > 3)
  int result = modbus_set_response_timeout(ctx_, sec, usec);
  return (result == 0);
#else
  struct timeval response_timeout;
  response_timeout.tv_sec = sec;
  response_timeout.tv_usec = usec;
  modbus_set_response_timeout(ctx_, &response_timeout);
  return true;
#endif
}

bool ModbusRtu::SelectDevice(uint8_t device_id) {
  if (ctx_ == nullptr) {
    XLOG_ERROR("Modbus context not initialized");
    return false;
  }
  int ret = modbus_set_slave(ctx_, device_id);
  if (ret != 0) {
    XLOG_ERROR("Failed to select slave device");
    return false;
  }
  return true;
}

int ModbusRtu::ReadCoils(uint8_t device_id, uint16_t addr, uint16_t quantity,
                         uint8_t *data) {
  return modbus_read_bits(ctx_, addr, quantity, data);
}

int ModbusRtu::WriteSingleCoil(uint8_t device_id, uint16_t addr, int status) {
  return modbus_write_bit(ctx_, addr, status);
}

int ModbusRtu::WriteMultipleCoils(uint8_t device_id, uint16_t addr,
                                  uint16_t quantity, const uint8_t *data) {
  return modbus_write_bits(ctx_, addr, quantity, data);
}

int ModbusRtu::ReadDiscreteInputs(uint8_t device_id, uint16_t addr,
                                  uint16_t quantity, uint8_t *data) {
  return modbus_read_input_bits(ctx_, addr, quantity, data);
}

int ModbusRtu::ReadInputRegisters(uint8_t device_id, uint16_t addr,
                                  uint16_t quantity, uint16_t *data) {
  return modbus_read_input_registers(ctx_, addr, quantity, data);
}

int ModbusRtu::ReadHoldingRegisters(uint8_t device_id, uint16_t addr,
                                    uint16_t quantity, uint16_t *data) {
  return modbus_read_registers(ctx_, addr, quantity, data);
}

int ModbusRtu::WriteSingleRegister(uint8_t device_id, uint16_t addr,
                                   uint16_t value) {
  return modbus_write_register(ctx_, addr, value);
}

int ModbusRtu::WriteMultipleRegisters(uint8_t device_id, uint16_t addr,
                                      uint16_t quantity, const uint16_t *data) {
  return modbus_write_registers(ctx_, addr, quantity, data);
}
}  // namespace xmotion