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

  Ddsm210Array array("/dev/ttyUSB0");
  uint8_t motor_id = 1;
  array.RegisterMotor(motor_id);

  if (!array.Connect()) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  array.RequestModeFeedback(motor_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "Current mode: " << static_cast<int>(array.GetMode(motor_id))
            << std::endl;

  bool variable_speed = false;
  int speed = 10;
  bool increase = true;
  for (int i = 0; i < 5000 && keep_running; ++i) {
    if (variable_speed) {
      if (i % 10 == 0) {
        if (increase) {
          speed += 1;
        } else {
          speed -= 1;
        }
        if (speed >= 210) {
          increase = false;
        } else if (speed <= -210) {
          increase = true;
        }
        std::cout << "Target speed: " << speed << std::endl;
      }
    }
    array.SetSpeed(motor_id, speed);
    array.RequestOdometryFeedback(motor_id);
    std::cout << "Speed: " << array.GetSpeed(motor_id)
              << ", encoder count: " << array.GetEncoderCount(motor_id)
              << " , position: " << array.GetPosition(motor_id) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  array.ApplyBrake(motor_id);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  array.Disconnect();

  return 0;
}