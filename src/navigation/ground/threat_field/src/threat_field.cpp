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

ThreatField::ThreatField(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap) : road_map_(rmap), traffic_map_(tmap)
{
}

void ThreatField::AddVehicleEstimations(std::vector<VehicleEstimation> ests)
{
    for (auto &est : ests)
        vehicles_.insert(std::make_pair(est.id_, est));
}

void ThreatField::SetupThreatField(std::shared_ptr<TrafficChannel> ego_chn)
{
    ego_channel_ = ego_chn;

    conflicting_lanes_ = road_map_->FindConflictingLanes(ego_channel_->lanes_);

    std::cout << "conflicting lanes:" << std::endl;
    for(auto& lane : conflicting_lanes_)
        std::cout << lane << std::endl;

    std::cout << "vehicle number: " << vehicles_.size() << std::endl;
    for (auto &entry : vehicles_)
    {
        if (!CheckInConflict(entry.second))
            continue;

        std::vector<std::shared_ptr<CollisionThreat>> threats;

        for (auto chn : entry.second.GetOccupiedChannels())
            threats.push_back(std::make_shared<CollisionThreat>(entry.second, chn));
        threats_.insert(std::make_pair(entry.second.id_, threats));
    }
}

bool ThreatField::CheckInConflict(VehicleEstimation veh)
{
    auto occupied_lanes = road_map_->FindOccupiedLaneletNames({veh.GetPose().position.x, veh.GetPose().position.y});
    std::cout << "occupied lanes of vehicle " << veh.id_ << " : " << std::endl;
    for(auto& lane : occupied_lanes)
        std::cout << lane << std::endl; 

    for (auto &olane : occupied_lanes)
    {
        for (auto &clane : conflicting_lanes_)
        {
            if (olane == clane)
                return true;
        }
    }

    return false;
}

void ThreatField::UpdateThreatField(int32_t t_k)
{
    for (auto &vehicle_threats : threats_)
    {
        for (auto threat : vehicle_threats.second)
            threat->UpdateOccupancyDistribution(t_k);
    }
}

std::vector<VehicleEstimation> ThreatField::GetAllVehicleEstimations()
{
    std::vector<VehicleEstimation> vehs;

    for (auto &veh_entry : vehicles_)
    {
        vehs.push_back(veh_entry.second);
    }

    return vehs;
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

double ThreatField::operator()(double x, double y)
{
    double threat = 0.0;
    for (auto &threat_entry : threats_)
        for (auto sub : threat_entry.second)
            threat += (*sub.get())(x, y);
    return threat;
}

Point2d ThreatField::GetThreatCenter()
{
    Point2d pos(0, 0);
    for (auto &threat_entry : threats_)
        for (auto sub : threat_entry.second)
        {
            auto c = sub->GetThreatCenter();
            pos.x += c.x;
            pos.y += c.y;
        }
    pos.x = pos.x / threats_.size();
    pos.y = pos.y / threats_.size();
    return pos;
}

double ThreatField::operator()(double x, double y, int32_t t_k)
{
    double threat = 0.0;
    for (auto &threat_entry : threats_)
        for (auto sub : threat_entry.second)
            threat += (*sub.get())(x, y, t_k);
    return threat;
}

Point2d ThreatField::GetThreatCenter(int32_t t_k)
{
    Point2d pos(0, 0);
    for (auto &threat_entry : threats_)
        for (auto sub : threat_entry.second)
        {
            auto c = sub->GetThreatCenter(t_k);
            pos.x += c.x;
            pos.y += c.y;
        }
    pos.x = pos.x / threats_.size();
    pos.y = pos.y / threats_.size();
    return pos;
}