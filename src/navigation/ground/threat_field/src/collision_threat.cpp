/* 
 * collision_threat.cpp
 * 
 * Created on: Nov 02, 2018 01:40
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/collision_threat.hpp"

using namespace librav;

CollisionThreat::CollisionThreat(VehicleEstimation est, std::shared_ptr<TrafficChannel> chn) : vehicle_est_(est), traffic_chn_(chn)
{
    SetupPredictionModel();
}

void CollisionThreat::SetupPredictionModel()
{
}