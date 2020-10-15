/* 
 * threat_field.cpp
 * 
 * Created on: Nov 04, 2018 04:22
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "prediction/threat_field.hpp"

#include <tbb/tbb.h>

using namespace ivnav;

ThreatField::ThreatField(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap) : road_map_(rmap), traffic_map_(tmap)
{
}

void ThreatField::AddVehicleEstimations(std::vector<VehicleEstimation> ests)
{
    for (auto &est : ests)
        vehicles_.insert(std::make_pair(est.id_, est));
}

void ThreatField::SetupThreatField(Pose2d ego_pose, std::shared_ptr<TrafficChannel> ego_chn)
{
    ego_pose_ = ego_pose;
    ego_channel_ = ego_chn;

    conflicting_lanes_ = road_map_->FindConflictingLanes(ego_channel_->lanes_);

    // for (auto &entry : vehicles_)
    // {
    //     if (!CheckInConflict(entry.second))
    //         continue;

    //     std::shared_ptr<VehicleThreat> threat = std::make_shared<VehicleThreat>(entry.second, traffic_map_);

    //     threats_.insert(std::make_pair(entry.second.id_, threat));
    // }

    std::vector<int32_t> veh_ids;
    for (auto &entry : vehicles_)
        veh_ids.push_back(entry.first);

    const auto &init_vehicle_threat = [this, &veh_ids](size_t i) {
        if (!this->CheckInConflict(this->vehicles_[veh_ids[i]]))
            return;
        std::shared_ptr<VehicleThreat> threat = std::make_shared<VehicleThreat>(this->vehicles_[veh_ids[i]], this->traffic_map_);
        this->step_increment_ = threat->GetPrecitionStepIncrement();
        this->threats_.insert(std::make_pair(this->vehicles_[veh_ids[i]].id_, threat));
    };
    tbb::parallel_for(size_t(0), size_t(veh_ids.size()), init_vehicle_threat);
}

bool ThreatField::CheckInConflict(VehicleEstimation veh)
{
    auto occupied_lanes = road_map_->FindOccupiedLaneletNames({veh.GetPose().position.x, veh.GetPose().position.y});

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

void ThreatField::ComputeThreatField(int32_t t_k)
{
    for (auto &vehicle_threat : threats_)
        vehicle_threat.second->ComputeOccupancyDistribution(t_k);
}

std::vector<VehicleEstimation> ThreatField::GetAllVehicleEstimations()
{
    std::vector<VehicleEstimation> vehs;

    for (auto &veh_entry : vehicles_)
        vehs.push_back(veh_entry.second);

    return vehs;
}

std::vector<std::shared_ptr<VehicleThreat>> ThreatField::GetAllCollisionThreats()
{
    std::vector<std::shared_ptr<VehicleThreat>> threats;

    for (auto &vehicle_threats : threats_)
        threats.push_back(vehicle_threats.second);

    return threats;
}

ThreatField::ThreatComponent ThreatField::GetThreatComponentAt(double x, double y, int32_t t_k)
{
    ThreatComponent comp;

    for (auto &threat_entry : threats_)
    {
        double threat = (*threat_entry.second.get())(x, y, t_k);
        comp.insert(std::make_pair(threat_entry.first, threat));
    }

    return comp;
}

double ThreatField::operator()(double x, double y, int32_t t_k)
{
    double threat = 0.0;
    for (auto &threat_entry : threats_)
        threat += (*threat_entry.second.get())(x, y, t_k);
    return threat;
}

Point2d ThreatField::GetThreatCenter(int32_t t_k)
{
    Point2d pos(0, 0);
    for (auto &threat_entry : threats_)
    {
        auto c = threat_entry.second->GetThreatCenter(t_k);
        pos.x += c.x;
        pos.y += c.y;
    }
    pos.x = pos.x / threats_.size();
    pos.y = pos.y / threats_.size();
    return pos;
}
