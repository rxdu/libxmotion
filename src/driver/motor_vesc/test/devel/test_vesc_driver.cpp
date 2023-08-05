/*
 * test_vesc_driver.cpp
 *
 * Created on 6/30/22 10:31 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "motor_vesc/vesc_can_interface.hpp"

using namespace xmotion;

void VescStateUpdatedCallback(const StampedVescState &state) {
  //  std::cout << "voltage input: " << state.state.voltage_input << ", "
  //            << "temp pcb: " << state.state.temperature_pcb << ", "
  //            << "current motor: " << state.state.current_motor << ", "
  //            << "current input: " << state.state.current_input << ", "
  //            << "speed: " << state.state.speed << ", "
  //            << "duty cycle: " << state.state.duty_cycle << ", "
  //            << "charge drawn: " << state.state.charge_drawn << ", "
  //            << "charge regen: " << state.state.charge_regen << ", "
  //            << "charge drawn: " << state.state.energy_drawn << ", "
  //            << "charge regen: " << state.state.energy_regen << ", "
  //            << "displacement: " << state.state.displacement << ", "
  //            << "dist traveled: " << state.state.distance_traveled
  //            << std::endl;
}

int main(int argc, char *argv[]) {
  VescCanInterface vesc;
  vesc.SetStateUpdatedCallback(VescStateUpdatedCallback);
  if (!vesc.Connect("can0", 0x68)) {
    std::cerr << "failed to connect" << std::endl;
  }

  //   float servo_pos = 0.5;
  //   bool increase = true;
  while (1) {
    // if (increase)
    //   servo_pos += 0.1;
    // else
    //   servo_pos += -0.1;
    // if (servo_pos >= 1.0) {
    //   increase = false;
    // } else if (servo_pos <= 0) {
    //   increase = true;
    // }
    // vesc.SetServo(servo_pos);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // printf("setting: %f\n", servo_pos);

    // vesc.SetSpeed(2000);
    vesc.SetServo(0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return 0;
}