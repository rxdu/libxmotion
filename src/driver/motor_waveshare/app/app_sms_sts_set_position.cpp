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
  int id = 1;
  int position = 0;
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <motor_id> <position>" << std::endl;
    std::cout << "position: 0-360" << std::endl;
    return -1;
  }
  id = std::stoi(argv[1]);
  position = std::stoi(argv[2]);

  if (position < 0 || position > 360) {
    std::cout << "position should be in range 0-360" << std::endl;
    return -1;
  }

  SmsStsServo servo(id);

  if (!servo.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  servo.SetPosition(position);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  return 0;
}