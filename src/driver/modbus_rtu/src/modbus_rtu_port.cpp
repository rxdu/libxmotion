/*
 * modbus_rtu_port.cpp
 *
 * Created on 4/18/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "modbus_rtu/modbus_rtu_port.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
ModbusRtuPort::ModbusRtuPort() {}

bool ModbusRtuPort::Open(const std::string &port_name, int baud_rate,
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

void ModbusRtuPort::Close() {
  if (ctx_ != nullptr) {
    modbus_close(ctx_);
    modbus_free(ctx_);
  }
}

bool ModbusRtuPort::IsOpened() const { return (ctx_ != nullptr); }

bool ModbusRtuPort::SetResponseTimeout(int sec, int usec) {
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

bool ModbusRtuPort::SelectDevice(uint8_t device_id) {
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

int ModbusRtuPort::ReadCoils(uint8_t device_id, uint16_t addr,
                             uint16_t quantity, uint8_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_read_bits(ctx_, addr, quantity, data);
}

int ModbusRtuPort::WriteSingleCoil(uint8_t device_id, uint16_t addr,
                                   int status) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_write_bit(ctx_, addr, status);
}

int ModbusRtuPort::WriteMultipleCoils(uint8_t device_id, uint16_t addr,
                                      uint16_t quantity, const uint8_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_write_bits(ctx_, addr, quantity, data);
}

int ModbusRtuPort::ReadDiscreteInputs(uint8_t device_id, uint16_t addr,
                                      uint16_t quantity, uint8_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_read_input_bits(ctx_, addr, quantity, data);
}

int ModbusRtuPort::ReadInputRegisters(uint8_t device_id, uint16_t addr,
                                      uint16_t quantity, uint16_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_read_input_registers(ctx_, addr, quantity, data);
}

int ModbusRtuPort::ReadHoldingRegisters(uint8_t device_id, uint16_t addr,
                                        uint16_t quantity, uint16_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_read_registers(ctx_, addr, quantity, data);
}

int ModbusRtuPort::WriteSingleRegister(uint8_t device_id, uint16_t addr,
                                       uint16_t value) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_write_register(ctx_, addr, value);
}

int ModbusRtuPort::WriteMultipleRegisters(uint8_t device_id, uint16_t addr,
                                          uint16_t quantity,
                                          const uint16_t *data) {
  if (ctx_ == nullptr || !SelectDevice(device_id)) return -1;
  return modbus_write_registers(ctx_, addr, quantity, data);
}
}  // namespace xmotion