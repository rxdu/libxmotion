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

#include "vesc_driver/vesc_can_interface.hpp"

using namespace robosw;

void VescStateUpdatedCallback(const StampedVescState &state) {
  std::cout << "voltage input: " << state.state.voltage_input << ", "
            << "temp pcb: " << state.state.temperature_pcb << ", "
            << "current motor: " << state.state.current_motor << ", "
            << "current input: " << state.state.current_input << ", "
            << "speed: " << state.state.speed << ", "
            << "duty cycle: " << state.state.duty_cycle << ", "
            << "charge drawn: " << state.state.charge_drawn << ", "
            << "charge regen: " << state.state.charge_regen << ", "
            << "charge drawn: " << state.state.energy_drawn << ", "
            << "charge regen: " << state.state.energy_regen << ", "
            << "displacement: " << state.state.displacement << ", "
            << "dist traveled: " << state.state.distance_traveled
            << std::endl;
}

int main(int argc, char *argv[]) {
  VescCanInterface vesc;
  vesc.SetStateUpdatedCallback(VescStateUpdatedCallback);
  if (!vesc.Connect("can0", 0x68)) {
    std::cerr << "failed to connect" << std::endl;
  }

  while (1) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}