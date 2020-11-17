/* 
 * bicycle_model.cpp
 * 
 * Created on: Mar 20, 2018 17:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "model/bicycle_model.hpp"

#include <cstdint>
#include <cmath>

using namespace rnav;

BicycleKinematics::BicycleKinematics(control_t u) : u_(u)
{
}

// x1 = x, x2 = y, x3 = v, x4 = theta
void BicycleKinematics::operator()(const asc::state_t &x, asc::state_t &xd, const double)
{
    xd[0] = x[2] * std::cos(x[3]);
    xd[1] = x[2] * std::sin(x[3]);
    xd[2] = u_.a;
    xd[3] = x[2] / L * std::tan(u_.delta);
}