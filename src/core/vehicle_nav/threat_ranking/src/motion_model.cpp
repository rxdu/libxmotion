/* 
 * motion_model.cpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/motion_model.hpp"

using namespace librav;

MotionModel::MotionModel(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    ConstructLineNetwork();
}

void MotionModel::ConstructLineNetwork()
{
    line_network_ = std::make_shared<Graph_t<LaneBlock>>();

    for (auto &source : road_map_->traffic_sources_)
        for (auto &sink : road_map_->traffic_sinks_)
        {
            auto path = road_map_->FindShortestRoute(source, sink);
            if (!path.empty())
            {
                for (auto it = path.begin(); it != path.end() - 1; ++it)
                {
                    int32_t src_id = *it;
                    int32_t dst_id = *(it + 1);

                    std::string src_name = road_map_->GetLaneletNameFromID(src_id);
                    std::string dst_name = road_map_->GetLaneletNameFromID(dst_id);

                    LaneBlock src_bk(src_id, src_name);
                    LaneBlock dst_bk(dst_id, dst_name);

                    src_bk.center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk.center_line = road_map_->GetLaneCenterLine(dst_name);

                    line_network_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

void MotionModel::MergePointsToNetwork()
{
    int32_t ignored_pt_num = 0;

    for (auto &est : ests_)
    {
        // find occupied lanelets
        auto lanelet_ids = road_map_->OccupiedLanelet(CartCooridnate(est.position_x, est.position_y));

        if (lanelet_ids.empty())
        {
            ++ignored_pt_num;
            continue;
        }

        std::cout << "occupied lanelet: " << lanelet_ids.size() << std::endl;

        auto pos = road_map_->coordinate_.ConvertToGridPixel(CartCooridnate(est.position_x, est.position_y));
        std::cout << "(x,y): " << pos << std::endl;

        MotionPoint pt(est);
        pt.SetGridCoordinate(pos);
        points_.push_back(pt);

        // find the closest center line
        for (auto &id : lanelet_ids)
        {
            std::cout << "occupied lanes: " << road_map_->GetLaneletNameFromID(id) << std::endl;
        }

        // identify chains to be maintained
    }
}

void MotionModel::FindMotionChains(std::string start_lane)
{
}

void MotionModel::GenerateCollisionField()
{
    cfield_ = std::make_shared<CollisionField>(road_map_->grid_size_x_, road_map_->grid_size_y_);
    cfield_->SetOriginCoordinate(0, 0);

    int32_t pt_count = 0;
    for (auto &pt : points_)
    {
        auto tf_pt = std::make_shared<CollisionField::TrafficParticipantType>(road_map_->grid_size_x_, road_map_->grid_size_y_);
        tf_pt->SetPositionVelocity(pt.grid_position_.x, pt.grid_position_.y, pt.estimate_.velocity_x, pt.estimate_.velocity_y);
        cfield_->AddTrafficParticipant(pt_count++, tf_pt);
    }
    cfield_->UpdateCollisionField();
}

Eigen::MatrixXd MotionModel::GetThreatFieldVisMatrix()
{
    return road_map_->GetFullCenterLineGrid()->GetGridMatrix(true) + road_map_->GetFullDrivableAreaGrid()->GetGridMatrix(true) +
        cfield_->GetGridMatrix(true);
}