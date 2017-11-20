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
    position_ = init_pos_;
}

VehicleSim::VehicleSim(Position2Dd init_pos) : init_pos_(init_pos)
{
    position_ = init_pos_;
}

void VehicleSim::Update(double t, double dt)
{
    // hard coded for testing
    if(t < 23)
        velocity_ = Position2Dd(0,5);
    else
        velocity_ = Position2Dd(5,0);

    position_.x = position_.x + velocity_.x * dt;
    position_.y = position_.y + velocity_.y * dt;
}