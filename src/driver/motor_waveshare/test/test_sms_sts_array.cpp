/*
 * @file test_sms_sts_array.cpp
 * @date 10/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "motor_waveshare/sms_sts_servo_array.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  SmsStsServoArray servo("/dev/ttyUSB0");
  servo.SetPositionOffset(180);
  servo.SetDefaultPosition(0);

  std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (auto id : ids) {
    servo.RegisterMotor(id);
  }

  if (!servo.Connect()) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  servo.SetPositions({45, 45, 45, 45});
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  servo.SetPositions({0, 0, 0, 0});
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  servo.SetPositions({-45, -45, -45, -45});
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  servo.SetPositions({0, 0, 0, 0});
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  return 0;
}