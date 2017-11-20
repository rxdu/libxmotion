/* 
 * vehicle_sim.cpp
 * 
 * Created on: Nov 20, 2017 10:25
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "traffic/vehicle_sim.h"

using namespace librav;

VehicleSim::VehicleSim() : init_pos_(Position2Dd(0, 0))
{
}

VehicleSim::VehicleSim(Position2Dd init_pos) : init_pos_(init_pos)
{
}