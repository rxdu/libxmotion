/* 
 * car_longitudinal_model.cpp
 * 
 * Created on: Oct 30, 2018 05:54
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "model/car_longitudinal_model.hpp"

using namespace librav;

CarLongitudinalModel::CarLongitudinalModel(control_t u) : u_(u)
{
}

// x1 = s, x2 = v
void CarLongitudinalModel::operator()(const asc::state_t &x, asc::state_t &xd, const double)
{
    // s_dot
    if (x[1] > v_max)
        xd[0] = v_max;
    else if (x[1] < 0)
        xd[0] = 0;
    else
        xd[0] = x[1];

    // v_dot 
    if ((x[1] > 0 && x[1] <= v_sw) || (u_ <= 0))
        xd[1] = a_max * u_;
    else if ((x[1] > v_sw) && (u_ > 0))
        xd[1] = a_max * v_sw / x[1] * u_;
    else if (x[1] <= 0)
        xd[1] = 0;
}