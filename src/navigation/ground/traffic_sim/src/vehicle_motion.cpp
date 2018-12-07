/* 
 * vehicle_motion.cpp
 * 
 * Created on: Nov 06, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_sim/vehicle_motion.hpp"

using namespace librav;

VehicleMotion::VehicleMotion(VehicleInfo info) : vehicle_info_(info)
{
}

VehicleState VehicleMotion::GetStateAt(double t)
{
    switch (vehicle_info_.drive_mode)
    {
    case DriveMode::ConstantSpeed:
        return PropagateConstSpeedModel(t);
    }
}

VehicleState VehicleMotion::PropagateConstSpeedModel(double t)
{
    double distance = vehicle_info_.init_s + vehicle_info_.init_speed * t;

    if (distance > vehicle_info_.channel->GetLength())
    {
        distance = vehicle_info_.channel->GetLength();
        out_of_scope_ = true;
    }

    auto cpoint = vehicle_info_.channel->ConvertToCurvePoint({distance, 0});

    std::cout << "distance: " << distance << " position: " << cpoint.x << " , " << cpoint.y << std::endl;

    return VehicleState({cpoint.x, cpoint.y, cpoint.theta}, vehicle_info_.init_speed);
}