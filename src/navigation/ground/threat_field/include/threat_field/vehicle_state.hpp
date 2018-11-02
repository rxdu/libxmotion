/* 
 * vehicle_state.hpp
 * 
 * Created on: Nov 02, 2018 01:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <cstdint>
#include <functional>

namespace librav
{
struct VehicleState
{
  VehicleState() : position_x(0), position_y(0), velocity(0) {}
  VehicleState(double px, double py, double v, double v_var) : position_x(px), position_y(py), velocity(v) {}

  double position_x;
  double position_y;
  double velocity;
  double vel_var;
  int32_t time_stamp;

  std::function<double(double, double)> threat_func;

  double GetThreatValue(double x, double y) { return threat_func(x, y); }
};
} // namespace librav
#endif /* VEHICLE_STATE_HPP */
