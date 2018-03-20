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
class LongitudinalDynamics
{
  public:
    LongitudinalDynamics(double u);

    // x1 = s, x2 = v
    void operator()(const asc::state_t &x, asc::state_t &xd, const double);

  private:
    double u_ = 0.0;
    static constexpr double v_sw = 7.3;
    static constexpr double a_max = 7;
};
}

#endif /* VEHICLE_DYNAMICS_HPP */
