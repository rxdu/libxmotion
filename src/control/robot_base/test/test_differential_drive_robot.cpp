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
#include <cmath>

#include "logging/xlogger.hpp"

#include "input_hid/joystick.hpp"

#include "modbus_rtu/modbus_rtu_port.hpp"
#include "motor_akelc/motor_akelc.hpp"
#include "motor_akelc/motor_akelc_modbus.hpp"

#include "robot_base/differential_drive_robot.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  int index = 0;
  if (argc == 2) {
    index = std::atoi(argv[1]);
  } else {
    std::cout << "Usage: ./bin/app_tbot_base_control <joystick_index>"
              << std::endl;
    return -1;
  }

  // set up joystick
  Joystick js(index);
  if (!js.Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return -1;
  }

  // set up robot base
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

  std::cout << "Both controllers are reachable." << std::endl;

  DifferentialDriveRobot::Config config;
  auto left_motor = std::make_shared<MotorAkelc>(left_motor_impl);
  auto right_motor = std::make_shared<MotorAkelc>(right_motor_impl);

  //  left_motor->SetSpeed(100);
  //  right_motor->SetSpeed(100);

  config.wheel_radius = 0.0188;
  config.track_width = 0.185;
  config.linear_vel_deadband = 0.1;
  config.angular_vel_deadband = 0.05;
  config.reverse_left_wheel = true;
  config.left_actuator_group = std::make_shared<SpeedActuatorGroup>(
      std::vector<std::shared_ptr<MotorControllerInterface>>{left_motor});
  config.right_actuator_group = std::make_shared<SpeedActuatorGroup>(
      std::vector<std::shared_ptr<MotorControllerInterface>>{right_motor});

  DifferentialDriveRobot robot(config);

  std::cout << "Robot base initialized successfully" << std::endl;

  // mapping params from joystick axes to robot motion command
  double max_linear_vel = 1.0;
  double max_angular_vel = 1.5;

  while (js.IsOpened()) {
    //    std::cout << "Axes X: " << js.GetAxisState(JsAxis::kX).value
    //              << ", Axes Y: " << js.GetAxisState(JsAxis::kY).value
    //              << ", Axes Z: " << js.GetAxisState(JsAxis::kZ).value
    //              << ", Axes RX: " << js.GetAxisState(JsAxis::kRX).value
    //              << ", Axes RY: " << js.GetAxisState(JsAxis::kRY).value
    //              << ", Axes RZ: " << js.GetAxisState(JsAxis::kRZ).value
    //              << std::endl;

    //    if (js.GetButtonState(JsButton::kMode)) {
    //      js.SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    //    } else {
    //      js.SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    //    }

    // convert from joystick axes to robot motion command
    double linear_vel = -js.GetAxisState(JsAxis::kY).value;
    double angular_vel = -js.GetAxisState(JsAxis::kRX).value;
    // map linear vel from [-1,1] to [-max_linear_vel, max_linear_vel]
    linear_vel *= max_linear_vel;
    // map angular vel from [-1,1] to [-max_angular_vel, max_angular_vel]
    angular_vel *= max_angular_vel;
//    std::cout << "linear_vel: " << linear_vel
//              << ", angular_vel: " << angular_vel << std::endl;

    robot.SetMotionCommand(linear_vel, angular_vel);

    double actual_linear_vel, actual_angular_vel;
    robot.GetMotionState(actual_linear_vel, actual_angular_vel);
    //    std::cout << "actual_linear_vel: " << actual_linear_vel
    //              << ", actual_angular_vel: " << actual_angular_vel <<
    //              std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  //  robot.SetMotionCommand(0.0, 0.0);

  return 0;
}