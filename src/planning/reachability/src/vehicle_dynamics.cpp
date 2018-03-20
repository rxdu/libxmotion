/* 
 * vehicle_dynamics.cpp
 * 
 * Created on: Mar 20, 2018 17:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/vehicle_dynamics.hpp"

using namespace librav;

LongitudinalDynamics::LongitudinalDynamics(double u) : u_(u)
{
}

// x1 = s, x2 = v
void LongitudinalDynamics::operator()(const asc::state_t &x, asc::state_t &xd, const double)
{
    xd[0] = x[1];

    if (x[1] <= 0)
        xd[1] = 0;
    else if (x[1] > v_sw && u_ > 0)
        xd[1] = a_max * v_sw / x[1] * u_;
    else
        xd[1] = a_max * u_;
}