/*
 * @file ws_ddsm210_set_motor_id.cpp
 * @date 10/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "motor_waveshare/ddsm_210.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  int id = 1;
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <motor_id>" << std::endl;
    return -1;
  }
  id = std::stoi(argv[1]);

  Ddsm210 motor(1);

  if (!motor.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  std::cout << "Setting motor id to " << id << std::endl;
  if (motor.SetMotorId(id, 1000)) {
    std::cout << "Motor id set successfully" << std::endl;
  } else {
    std::cout << "Failed to set motor id" << std::endl;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  motor.Disconnect();

  return 0;
}