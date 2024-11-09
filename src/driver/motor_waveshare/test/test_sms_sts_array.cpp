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
  SmsStsServoArray servo;
  servo.SetPositionOffset(180);
  servo.SetDefaultPosition(0);

  std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (auto id : ids) {
    servo.RegisterMotor(id);
  }

  if (!servo.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  std::unordered_map<uint8_t, float> positions;
  for (auto id : ids) {
    positions[id] = 45;
  }
  servo.SetPositions(positions);
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  for (auto id : ids) {
    positions[id] = -45;
  }
  servo.SetPositions(positions);
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  for (auto id : ids) {
    positions[id] = 0;
  }
  servo.SetPositions(positions);
  std::this_thread::sleep_for(std::chrono::microseconds(2187 * 1000));

  return 0;
}