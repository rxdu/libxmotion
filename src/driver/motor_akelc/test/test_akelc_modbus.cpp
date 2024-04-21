#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "modbus_rtu/modbus_rtu_port.hpp"
#include "motor_akelc/motor_akelc_modbus.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  std::cout << "Hello, World!" << std::endl;

  std::shared_ptr<ModbusRtuPort> port = std::make_shared<ModbusRtuPort>();

  if (port->Open("/dev/ttyUSB0", 115200, ModbusRtuPort::Parity::kEven,
                 ModbusRtuPort::DataBit::kBit8,
                 ModbusRtuPort::StopBit::kBit1)) {
    std::cout << "Port opened successfully" << std::endl;
  } else {
    std::cout << "Failed to open port" << std::endl;
    return -1;
  }

  std::shared_ptr<MotorAkelcModbus> motor =
      std::make_shared<MotorAkelcModbus>(port, 0x05);

  if (!motor->IsReachable()) {
    std::cout << "Controller not reachable." << std::endl;
    return -1;
  }
  std::cout << "Found device: " << motor->GetDeviceName() << std::endl;

  std::cout << "Driver temperature: " << motor->GetDriverTemperature()
            << std::endl;
  std::cout << "Driver input voltage: " << motor->GetDriverInputVoltage()
            << std::endl;
  std::cout << "Driver current: " << motor->GetDriverCurrent() << std::endl;
  std::cout << "Driver PWM: " << motor->GetDriverPwm() << std::endl;

  motor->SetTargetRpm(200);
  std::this_thread::sleep_for(std::chrono::seconds(15));
  motor->SetTargetRpm(0);

  return 0;
}
