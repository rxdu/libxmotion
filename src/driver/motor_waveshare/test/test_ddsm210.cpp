/*
 * @file test_ddsm210.cpp
 * @date 10/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>
#include <chrono>

#include "motor_waveshare/ddsm_210.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  Ddsm210 motor(1);

  if (!motor.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  for (int i = 0; i < 500; ++i) {
    motor.SetSpeed(10);
    //    std::cout << "Speed: " << motor.GetSpeed() << std::endl;
    std::cout << "speed: " << motor.GetSpeed() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  motor.ApplyBrake();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  motor.Disconnect();

  return 0;
}