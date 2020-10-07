/* 
 * vehicle_manager.cpp
 * 
 * Created on: Dec 06, 2018 07:55
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_sim/vehicle_manager.hpp"

using namespace autodrive;

VehicleManager::VehicleManager(std::shared_ptr<TrafficMap> tmap) : traffic_map_(tmap)
{
}

void VehicleManager::AddVehicles(const std::vector<VehicleInfo> &vehs)
{
    for (auto info : vehs)
    {
        auto tchn = traffic_map_->FindTrafficChannel(info.channel_name.first, info.channel_name.second);
        if (tchn != nullptr)
        {
            info.valid = true;
            info.channel = tchn;
            surrounding_vehicles_.emplace_back(info, VehicleMotion(info));
        }
    }
}

std::vector<VehicleState> VehicleManager::GetVehicleStatesAt(double t)
{
    std::vector<VehicleState> states;
    for (auto &sveh : surrounding_vehicles_)
    {
        if (t >= sveh.first.init_t && !sveh.second.IsOutOfScope())
            states.push_back(sveh.second.GetStateAt(t));
    }
    return states;
}