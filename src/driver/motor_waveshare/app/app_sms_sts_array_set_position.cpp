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
  float position = 0;
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <position>" << std::endl;
    std::cout << "position: 0-360" << std::endl;
    return -1;
  }
  position = std::stof(argv[1]);

  if (position < 0 || position > 360) {
    std::cout << "position should be in range 0-360" << std::endl;
    return -1;
  }

  std::vector<uint8_t> ids = {1, 2, 3, 4};
  SmsStsServo servo(ids);

  if (!servo.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  servo.SetPosition({position, position, position, position});
  std::cout << "set to " << position << std::endl;
  for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(2187 * 100));
    std::cout << "position: " << servo.GetPositions()[0] << std::endl;
  }

  return 0;
}