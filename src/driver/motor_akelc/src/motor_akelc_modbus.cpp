/*
 * motor_akelc_modbus.cpp
 *
 * Created on 4/20/24 11:52 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_akelc/motor_akelc_modbus.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
// constants
constexpr uint16_t AQMD2410NS_B3_DEVICE_ID = 0x0101;
constexpr uint16_t AQMD6020NS_A3_DEVICE_ID = 0x0102;

// state registers
constexpr uint16_t REGISTER_ADDRESS_DEVICE_ID = 0x0000;
constexpr uint16_t REGISTER_ADDRESS_DEVICE_NAME = 0x0002;
constexpr uint16_t REGISTER_ADDRESS_DRIVER_PWM = 0x0010;
constexpr uint16_t REGISTER_ADDRESS_DRIVER_CURRENT = 0x0011;
constexpr uint16_t REGISTER_ADDRESS_SWITCHING_FREQ = 0x0012;

constexpr uint16_t REGISTER_ADDRESS_BLOCKED_STATE = 0x0013;
constexpr uint16_t REGISTER_ADDRESS_ERROR_STATE = 0x0017;

constexpr uint16_t REGISTER_ADDRESS_DRIVER_TEMPERATURE = 0x001c;
constexpr uint16_t REGISTER_ADDRESS_DRIVER_INPUT_VOLTAGE = 0x001d;
constexpr uint16_t REGISTER_ADDRESS_DRIVER_ACTUAL_RPM_HIGH = 0x001e;
constexpr uint16_t REGISTER_ADDRESS_DRIVER_ACTUAL_RPM_LOW = 0x001f;

// control registers
constexpr uint16_t REGISTER_ADDRESS_CTRL_TARGET_SPEED = 0x0040;
constexpr uint16_t REGISTER_ADDRESS_CTRL_APPLY_BRAKE = 0x0042;
constexpr uint16_t REGISTER_ADDRESS_CTRL_RELEASE_BRAKE = 0x0044;
}  // namespace

MotorAkelcModbus::MotorAkelcModbus(std::shared_ptr<ModbusRtuInterface> port,
                                   int device_id)
    : ModbusRtuClient(port, device_id) {}

bool MotorAkelcModbus::IsReachable() {
  if (!port_) return false;
  int ret = port_->ReadHoldingRegisters(device_id_, REGISTER_ADDRESS_DEVICE_ID,
                                        1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DEVICE_ID, ret);
    return false;
  }
  XLOG_DEBUG_STREAM("Queried device ID: " << std::hex << buffer_[0]);
  return true;
}

std::string MotorAkelcModbus::GetDeviceName() {
  if (!port_) return "Unknown Device";
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_DEVICE_NAME, 8, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DEVICE_NAME, ret);
    return "Unknown Device";
  }
  return {reinterpret_cast<char*>(buffer_)};
}

bool MotorAkelcModbus::SetTargetSwitchingFreq(int16_t freq) {
  if (!port_) return false;
  buffer_[0] = freq;
  int ret = port_->WriteSingleRegister(
      device_id_, REGISTER_ADDRESS_CTRL_TARGET_SPEED, buffer_[0]);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to write register {}, error code: {}",
               REGISTER_ADDRESS_SWITCHING_FREQ, ret);
    return false;
  }
  return true;
}

int16_t MotorAkelcModbus::GetActualSwitchingFreq() {
  if (!port_) return -1;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_SWITCHING_FREQ, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_SWITCHING_FREQ, ret);
    return -1;
  }
  return static_cast<int16_t>(buffer_[0]);
}

bool MotorAkelcModbus::SetTargetRpm(int32_t rpm) {
  if (!port_) return false;
  if (rpm < -std::numeric_limits<int16_t>::max())
    rpm = -std::numeric_limits<int16_t>::max();
  if (rpm > std::numeric_limits<int16_t>::max())
    rpm = std::numeric_limits<int16_t>::max();
  buffer_[0] = static_cast<uint16_t>(static_cast<int16_t>(rpm));
  int ret = port_->WriteSingleRegister(
      device_id_, REGISTER_ADDRESS_CTRL_TARGET_SPEED, buffer_[0]);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to write register {}, error code: {}",
               REGISTER_ADDRESS_CTRL_TARGET_SPEED, ret);
    return false;
  }
  return true;
}

int32_t MotorAkelcModbus::GetActualRpm() {
  if (!port_) return 0;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_DRIVER_ACTUAL_RPM_HIGH, 2, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DRIVER_ACTUAL_RPM_HIGH, ret);
    return 0;
  }
  return static_cast<int32_t>((static_cast<uint32_t>(buffer_[0]) << 16) |
                              static_cast<uint32_t>(buffer_[1]));
}

bool MotorAkelcModbus::ApplyBrake(float percentage) {
  if (!port_) return false;
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  buffer_[0] = static_cast<uint16_t>(percentage * 10);
  int ret = port_->WriteSingleRegister(
      device_id_, REGISTER_ADDRESS_CTRL_APPLY_BRAKE, buffer_[0]);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to write register {}, error code: {}",
               REGISTER_ADDRESS_CTRL_APPLY_BRAKE, ret);
    return false;
  }
  return true;
}

bool MotorAkelcModbus::ReleaseBrake() {
  if (!port_) return false;
  int ret = port_->WriteSingleRegister(device_id_,
                                       REGISTER_ADDRESS_CTRL_RELEASE_BRAKE, 1);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to write register {}, error code: {}",
               REGISTER_ADDRESS_CTRL_RELEASE_BRAKE, ret);
    return false;
  }
  return true;
}

double MotorAkelcModbus::GetDriverCurrent() {
  if (!port_) return 0;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_DRIVER_CURRENT, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DRIVER_CURRENT, ret);
    return 0;
  }
  return buffer_[0] * 0.0001f;
}

double MotorAkelcModbus::GetDriverPwm() {
  if (!port_) return 0;
  int ret = port_->ReadHoldingRegisters(device_id_, REGISTER_ADDRESS_DRIVER_PWM,
                                        1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DRIVER_PWM, ret);
    return 0;
  }
  return buffer_[0] * 0.001f;
}

double MotorAkelcModbus::GetDriverTemperature() {
  if (!port_) return 0;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_DRIVER_TEMPERATURE, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DRIVER_TEMPERATURE, ret);
    return 0;
  }
  return static_cast<int16_t>(buffer_[0]) * 0.1f;
}

double MotorAkelcModbus::GetDriverInputVoltage() {
  if (!port_) return 0;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_DRIVER_INPUT_VOLTAGE, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_DRIVER_INPUT_VOLTAGE, ret);
    return 0;
  }
  return buffer_[0] * 0.1f;
}

bool MotorAkelcModbus::IsMotorBlocked() {
  if (!port_) return false;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_BLOCKED_STATE, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_BLOCKED_STATE, ret);
    return false;
  }
  return buffer_[0] != 0;
}

MotorAkelcInterface::ErrorCode MotorAkelcModbus::GetErrorCode() {
  if (!port_) return ErrorCode::kNoError;
  int ret = port_->ReadHoldingRegisters(
      device_id_, REGISTER_ADDRESS_ERROR_STATE, 1, buffer_);
  if (ret < 0) {
    XLOG_ERROR("[Akelc Modbus] Failed to read register {}, error code: {}",
               REGISTER_ADDRESS_ERROR_STATE, ret);
    return ErrorCode::kUnknown;
  }
  return static_cast<ErrorCode>(buffer_[0]);
}
}  // namespace xmotion