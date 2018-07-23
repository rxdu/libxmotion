/* 
 * vehicle_dynamics.cpp
 * 
 * Created on: Mar 20, 2018 17:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/vehicle_dynamics.hpp"

#include <cstdint>

using namespace librav;

LongitudinalDynamics::LongitudinalDynamics(control_t u) : u_(u)
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

//////////////////////////////////////////////////////////////////

SteeringKinematics::SteeringKinematics(control_t u) : u_(u)
{
}

// x1 = x, x2 = y, x3 = v, x4 = theta
void SteeringKinematics::operator()(const asc::state_t &x, asc::state_t &xd, const double)
{
    xd[0] = x[2] * std::cos(x[3]);
    xd[1] = x[2] * std::sin(x[3]);
    xd[2] = u_[0];
    xd[3] = x[2] / L * u_[1];
}