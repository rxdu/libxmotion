/*
 * test_tbot.cpp
 *
 * Created on 4/21/24 10:41 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "modbus_rtu/modbus_rtu_port.hpp"
#include "motor_akelc/motor_akelc.hpp"
#include "motor_akelc/motor_akelc_modbus.hpp"

#include "robot_base/differential_drive_robot.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  std::cout << "Hello, TBOT!" << std::endl;

  std::shared_ptr<ModbusRtuPort> port = std::make_shared<ModbusRtuPort>();

  if (port->Open("/dev/ttyUSB0", 115200, ModbusRtuPort::Parity::kEven,
                 ModbusRtuPort::DataBit::kBit8,
                 ModbusRtuPort::StopBit::kBit1)) {
    std::cout << "Port opened successfully" << std::endl;
  } else {
    std::cout << "Failed to open port" << std::endl;
    return -1;
  }

  std::shared_ptr<MotorAkelcModbus> left_motor_impl =
      std::make_shared<MotorAkelcModbus>(port, 0x05);
  std::shared_ptr<MotorAkelcModbus> right_motor_impl =
      std::make_shared<MotorAkelcModbus>(port, 0x06);

  if (!left_motor_impl->IsReachable() || !right_motor_impl->IsReachable()) {
    std::cout << "Controller not reachable." << std::endl;
    return -1;
  }

  auto left_motor = std::make_shared<MotorAkelc>(left_motor_impl);
  auto right_motor = std::make_shared<MotorAkelc>(right_motor_impl);

  DifferentialDriveRobot robot(
      std::make_shared<SpeedActuatorGroup>(
          std::vector<std::shared_ptr<MotorControllerInterface>>{left_motor}),
      std::make_shared<SpeedActuatorGroup>(
          std::vector<std::shared_ptr<MotorControllerInterface>>{right_motor}));

  return 0;
}