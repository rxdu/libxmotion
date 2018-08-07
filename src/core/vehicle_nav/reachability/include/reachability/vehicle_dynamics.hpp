/* 
 * vehicle_dynamics.hpp
 * 
 * Created on: Mar 20, 2018 17:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_DYNAMICS_HPP
#define VEHICLE_DYNAMICS_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

namespace librav
{
// Reference: 
//  [1] Broadhurst, A., S. Baker, and T. Kanade. 2005. “Monte Carlo Road Safety Reasoning.” 
//      In IEEE Proceedings. Intelligent Vehicles Symposium, 2005., 319–24.
class LongitudinalDynamics
{
public:
  using control_t = double;
  LongitudinalDynamics(double u);

  // x1 = s, x2 = v
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

private:
  double u_ = 0.0;
  static constexpr double v_sw = 7.3;
  static constexpr double a_max = 7;
};

struct BicycleKinCtrlInput
{
  BicycleKinCtrlInput() : accel(0.0), steering(0.0) {}
  BicycleKinCtrlInput(double acc, double ster) : accel(acc), steering(ster) {}

  double accel;
  double steering;
};

class BicycleKinematics
{
public:
  using control_t = BicycleKinCtrlInput;
  BicycleKinematics(control_t u);

  // x1 = x, x2 = y, x3 = v, x4 = theta
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

private:
  control_t u_ = {0.0, 0.0};
  static constexpr double L = 2.4;
};
}

#endif /* VEHICLE_DYNAMICS_HPP */
