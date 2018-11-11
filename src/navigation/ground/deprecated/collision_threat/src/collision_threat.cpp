/* 
 * collision_threat.cpp
 * 
 * Created on: Aug 29, 2018 00:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "collision_threat/collision_threat.hpp"

using namespace librav;

DynamicThreatModel::DynamicThreatModel(std::shared_ptr<RoadMap> map, std::shared_ptr<TrafficMap> tmap) : road_map_(map), traffic_map_(tmap)
{
}

std::shared_ptr<CollisionField> DynamicThreatModel::EvaluateThreat(double ave_spd)
{
    std::string source = ego_flow_->GetFlowSource();
    std::string sink = ego_flow_->GetSingleSourceFlowSink();

    conflicting_flows_ = traffic_map_->FindConflictingFlows(source, sink);

    auto poses = traffic_map_->BackTrackCollision(ego_flow_, conflicting_flows_, ave_spd);

    GenerateCollisionRegions(poses, ave_spd);

    return GenerateCollisionField();
}

void DynamicThreatModel::GenerateCollisionRegions(const std::vector<FlowTrackPoint> &centers, double spd)
{
    regions_.clear();
    for (auto &center : centers)
        regions_.push_back(CollisionRegion(center.pose, spd, center.time_stamp));
}

std::shared_ptr<CollisionField> DynamicThreatModel::GenerateCollisionField()
{
    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(road_map_->xmin_, road_map_->xmax_, road_map_->ymin_, road_map_->ymax_);

    int32_t pt_count = 0;
    for (auto &pt : regions_)
    {
        GaussianVelocityThreat threat_model(pt.pose.x, pt.pose.y,
                                            pt.velocity_x, pt.velocity_y,
                                            pt.sigma_x, pt.sigma_y);
        std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(pt.pose.x, pt.pose.y,
                                                                                               pt.velocity_x, pt.velocity_y);
        // participant->id = pt.estimate_.id;
        participant->time_stamp = pt.time_stamp;
        participant->threat_func = threat_model;
        cfield->AddTrafficParticipant(pt_count++, participant);
    }

    return cfield;
}

double DynamicThreatModel::CalculateThreatLevel(std::shared_ptr<CollisionField> cfield, double posx, double posy, double velx, double vely)
{
    double cost = cfield->GetCollisionThreat(posx, posy);

    // Reference:
    // [1] https://thecuriousastronomer.wordpress.com/2014/06/26/what-does-a-1-sigma-3-sigma-or-5-sigma-detection-mean/
    std::cout << "cost : " << cost << std::endl;
    if (cost < 0.0013)
        return 0;

    // for(auto& region: regions_)
    for (int i = 0; i < cfield->GetTrafficParticipantNumber(); ++i)
    {
        auto part = cfield->GetTrafficParticipant(i);

        if (part->GetThreatValue(posx, posy) < 0.0013)
            continue;

        double err_x = part->position_x - posx;
        double err_y = part->position_y - posy;
        double distance = std::hypot(err_x, err_y);
        double nominal_speed = distance / part->time_stamp;
        // double sigma = region.sigma_x;
        // std::cout << "participant position: " <<

        std::cout << "distance: " << distance << " , nomial speed: " << nominal_speed << std::endl;
    }

    return cost;
}