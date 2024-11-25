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
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <motor_id>" << std::endl;
    return -1;
  }
  id = std::stoi(argv[1]);

  SmsStsServo servo(1);

  if (!servo.Connect("/dev/ttyACM0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  if (servo.SetMotorId(id)) {
    std::cout << "Set motor id to " << id << std::endl;
  } else {
    std::cout << "Failed to set motor id" << std::endl;
    return -1;
  }

  return 0;
}