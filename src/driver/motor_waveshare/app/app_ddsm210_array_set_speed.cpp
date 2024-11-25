/*
 * @file test_ddsm210_array.cpp
 * @date 10/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "motor_waveshare/ddsm_210_array.hpp"

using namespace xmotion;

bool keep_running = true;

int main(int argc, char **argv) {
  std::signal(SIGINT, [](int signum) { keep_running = false; });

  float speed = 0;
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <speed>" << std::endl;
    return -1;
  }
  speed = std::stof(argv[1]);

  Ddsm210Array array("/dev/ttyUSB0");
  uint8_t motor_id = 1;
  array.RegisterMotor(1);
  array.RegisterMotor(2);
  array.RegisterMotor(3);
  array.RegisterMotor(4);

  if (!array.Connect()) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  array.RequestModeFeedback(motor_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "Current mode: " << static_cast<int>(array.GetMode(motor_id))
            << std::endl;

  while (keep_running) {
    for (int i = 1; i <= 4; ++i) {
      array.SetSpeed(i, speed);
      array.RequestOdometryFeedback(i);
      std::cout << "Motor " << i << " speed: " << array.GetSpeed(i)
                << ", encoder count: " << array.GetEncoderCount(motor_id)
                << " , position: " << array.GetPosition(motor_id) << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  array.ApplyBrake(motor_id);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  array.Disconnect();

  return 0;
}