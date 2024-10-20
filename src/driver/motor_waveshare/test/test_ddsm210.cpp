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
#include <csignal>

#include "motor_waveshare/ddsm_210.hpp"

using namespace xmotion;

bool keep_running = true;

int main(int argc, char **argv) {
  std::signal(SIGINT, [](int signum) { keep_running = false; });

  Ddsm210 motor(1);

  if (!motor.Connect("/dev/ttyUSB0")) {
    std::cout << "Failed to connect to motor" << std::endl;
    return -1;
  }

  motor.RequestModeFeedback();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "Current mode: " << static_cast<int>(motor.GetMode())
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
    motor.SetSpeed(speed);
    motor.RequestOdometryFeedback();
    //    std::cout << "Speed feedback: " << motor.GetSpeed() << std::endl;
    std::cout << "Encoder count: " << motor.GetEncoderCount()
              << " , position: " << motor.GetPosition() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  motor.ApplyBrake();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  motor.Disconnect();

  return 0;
}