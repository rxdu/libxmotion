/* 
 * threat_field.cpp
 * 
 * Created on: Nov 04, 2018 04:22
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/threat_field.hpp"

using namespace librav;

void ThreatField::AddVehicleEstimations(std::vector<VehicleEstimation> ests)
{
    for (auto &est : ests)
        vehicles_.insert(std::make_pair(est.id_, est));
}

void ThreatField::SetupThreatField()
{
    std::cout << "vehicle number: " << vehicles_.size() << std::endl;
    for (auto &entry : vehicles_)
    {
        std::vector<std::shared_ptr<CollisionThreat>> threats;

        for (auto chn : entry.second.GetOccupiedChannels())
            threats.push_back(std::make_shared<CollisionThreat>(entry.second, chn));
        threats_.insert(std::make_pair(entry.second.id_, threats));
    }
}

void ThreatField::UpdateThreatField(int32_t t_k)
{
    for (auto &vehicle_threats : threats_)
    {
        for (auto threat : vehicle_threats.second)
            threat->UpdateOccupancyDistribution(t_k);
    }
}

std::vector<std::shared_ptr<CollisionThreat>> ThreatField::GetAllCollisionThreats()
{
    std::vector<std::shared_ptr<CollisionThreat>> threats;
    
    for (auto &vehicle_threats : threats_)
    {
        for (auto threat : vehicle_threats.second)
            threats.push_back(threat);
    }

    return threats;
}