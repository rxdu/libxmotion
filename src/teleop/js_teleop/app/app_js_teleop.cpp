/*
 * test_joystick.cpp
 *
 * Created on 6/1/23 8:46 PM
 * Description:
 *
 * Input: joystick config and joystick readings
 *   - Required key: left joystick, right joystick, right trigger
 * Output: command of type Twist to /cmd_vel
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include <thread>
#include <chrono>
#include <iostream>

#include "input_joystick/joystick.hpp"
#include "js_teleop/joystick_teleop.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  auto jds = Joystick::EnumberateJoysticks();
  for (auto jd : jds) {
    std::cout << "Joystick index: " << jd.index << std::endl;
    std::cout << "Joystick name: " << jd.name << std::endl;
  }
  if (jds.empty()) {
    std::cout << "No joystick found" << std::endl;
    return -1;
  }

  JoystickTeleop::Config config;
  config.jd = jds.front();
  JoystickTeleop teleop(config);

  if (teleop.Initialize()) {
    teleop.Run(100);
  }

  return 0;
}