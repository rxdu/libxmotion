/*
 * @file test_sms_sts.cpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "motor_waveshare/sms_sts_servo.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  std::vector<uint8_t> ids = {1, 2, 3, 4};
  SmsStsServo servo(ids);

  if (!servo.Connect("/dev/ttyACM0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  while (true) {
    servo.SetPosition({360, 360, 360, 360});
    std::cout << "set to 360" << std::endl;
    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::microseconds(2187 * 100));
      std::cout << "position: " << servo.GetPositions()[0] << std::endl;
    }
    // clang-format off
//    for (int i = 0; i < 10; i++) {
//      auto state = servo.GetState();
//      std::cout << "position: " << state.position << ", speed: " << state.speed
//                << ", load: " << state.load << ", voltage: " << state.voltage
//                << ", temperature: " << state.temperature
//                << ", current: " << state.current
//                << ", is_moving: " << std::boolalpha << state.is_moving
//                << std::endl;
//      std::this_thread::sleep_for(std::chrono::microseconds(2187 * 100));
//      std::cout << "sleeping" << std::endl;
//    }
    // clang-format on
    //    std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));
    servo.SetPosition({0, 0, 0, 0});
    std::cout << "set to 0" << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));
  }

  return 0;
}