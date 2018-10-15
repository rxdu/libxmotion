/* 
 * vehicle_sim.cpp
 * 
 * Created on: Nov 20, 2017 10:25
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "traffic_sim/vehicle_sim.hpp"

using namespace librav;

VehicleSim::VehicleSim(double x, double y, double spd) : pos_x_(x),pos_y_(y), speed_(spd)
{
}

void VehicleSim::Update(double t)
{
}