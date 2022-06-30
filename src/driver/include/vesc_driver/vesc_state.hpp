/* 
 * vesc_state.hpp
 *
 * Created on 6/30/22 10:36 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATE_HPP
#define ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATE_HPP

#include <cstdint>
#include <chrono>

namespace robosw {
using VescClock = std::chrono::steady_clock;
using VescTimepoint = VescClock::time_point;

enum VescFaultCode {
  kFaultCodeNone = 0,
  kFaultCodeOverVoltage = 1,
  kFaultCodeUnderVoltage = 2,
  kFaultCodeDrv8302 = 3,
  kFaultCodeAbsOverCurrent = 4,
  kFaultCodeOverTempFET = 5,
  kFaultCodeOverTempMotor = 6
};

struct VescState {
  double voltage_input = 0;  // input voltage (volt)
  double temperature_pcb = 0; // temperature of printed circuit board (degrees Celsius)
  double current_motor = 0;  // motor current (ampere)
  double current_input = 0;  // input current (ampere)
  double speed = 0;  // motor electrical speed (revolutions per minute)
  double duty_cycle = 0;  // duty cycle (0 to 1)
  double charge_drawn = 0;  // electric charge drawn from input (ampere-hour)
  double charge_regen = 0;  // electric charge regenerated to input (ampere-hour)
  double energy_drawn = 0;  // energy drawn from input (watt-hour)
  double energy_regen = 0;  // energy regenerated to input (watt-hour)
  double displacement = 0;  // net tachometer (counts)
  double distance_traveled = 0; // total tachnometer (counts)
  int32_t fault_code = 0;
};

enum class VescStateUpdatedField {
  kStatus1 = 0,
  kStatus2,
  kStatus3,
  kStatus4,
  kStatus5
};

struct StampedVescState {
  VescTimepoint time;
  VescStateUpdatedField field;
  VescState state;
};
}

#endif //ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATE_HPP
