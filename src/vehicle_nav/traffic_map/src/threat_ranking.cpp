/* 
 * threat_ranking.cpp
 * 
 * Created on: Aug 08, 2018 00:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_map/threat_ranking.hpp"

#include <string>
#include <cstdint>

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

ThreatRanking::ThreatRanking(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    // create motion model
    motion_model_ = std::make_shared<MotionModel>(road_map_);

    // create lattice planner
    lattice_manager_ = std::make_shared<LatticeManager>();
    lattice_manager_->LoadPrimitivesFromFile("/home/rdu/mp.1s20mph-3.data");
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.uniform.data");
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.three-level.data");
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.uniform-dense.data");
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.21-dense.data");
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/mp.20180808051727.data");

    planner_ = std::make_shared<LatticePlanner>(lattice_manager_, road_map_);
}

void ThreatRanking::AddStateEstimations(std::vector<MMStateEst> ests)
{
    motion_model_->SetVehicleStateEstimates(ests);
    motion_model_->MergePointsToNetwork();
}

void ThreatRanking::SetEgoDesiredPath(std::string start, std::string goal)
{
    ego_path_ = road_map_->FindShortestRouteName(start, goal);
    planner_->SetEgoPlanningRoute(ego_path_);
}

void ThreatRanking::SetEgoStartState(double x, double y, double theta)
{
    start_x_ = x;
    start_y_ = y;
    start_theta_ = theta;
}

void ThreatRanking::SetEgoGoalState(double x, double y, double theta)
{
    goal_x_ = x;
    goal_y_ = y;
    goal_theta_ = theta;
}

void ThreatRanking::Analyze()
{
    // find ego vehicle's path
    LatticeNode start(start_x_, start_y_, start_theta_);
    LatticeNode goal(goal_x_, goal_y_, goal_theta_);

    std::cout << "--------------------------------" << std::endl;
    std::cout << "Searching for path ... " << std::endl;

    stopwatch::StopWatch timer;
    // auto path = planner.Search(start, goal);
    path_ = planner_->AStarSearch(start, goal, 8, 10);
    std::cout << "finished in " << timer.mtoc() << " seconds" << std::endl;

    // lattice_manager_->SavePrimitivesToFile(path_, "path");

    if (!path_.empty())
    {
        auto full_path = planner_->ConvertPathToPolyline(path_);
        for (int32_t i = 0; i < path_.size(); ++i)
        {
            auto path_line = path_[i].ToPolyline();
            auto cfield = motion_model_->GeneratePredictedCollisionField(i+1.0);
            CalculateThreatExposure(path_line, cfield);
            // LightViz::ShowPathSegmentCollisionFieldWithRoadMap(path_line, cfield, road_map_);
            LightViz::ShowPathCollisionFieldWithRoadMap(path_line, full_path, cfield, road_map_,
                                                        true, 10, case_label_ + "-" + std::to_string(i), true);
        }
    }

    PrintCostInfo();
}

void ThreatRanking::CalculateThreatExposure(const Polyline &line, std::shared_ptr<CollisionField> cfield)
{
    for (std::size_t i = 0; i < line.GetPointNumer(); ++i)
    {
        auto pt = line.GetPoint(i);
        for (std::size_t j = 0; j < cfield->GetTrafficParticipantNumber(); ++j)
        {
            int32_t threat_id = cfield->GetTrafficParticipant(j)->id;
            double pt_cost = cfield->GetTrafficParticipant(j)->GetThreatValue(pt.x, pt.y);
            if (threat_cost_.find(threat_id) == threat_cost_.end())
                threat_cost_.insert(std::make_pair(threat_id, pt_cost));
            else
                threat_cost_[threat_id] += pt_cost;
        }
    }
}

void ThreatRanking::PrintCostInfo()
{
    std::map<double, int32_t, std::greater<double>> threat_cost_map;

    std::cout << "++++++++++++++++++++++++" << std::endl;
    for (auto &tc : threat_cost_)
    {
        threat_cost_map.insert(std::make_pair(tc.second, tc.first));
        std::cout << "traffic participant: " << tc.first << "  cost : " << tc.second << std::endl;
    }
    std::cout << "----> importance ranking: " << std::endl;
    for (auto &tc : threat_cost_map)
    {
        std::cout << "traffic participant: " << tc.second << "  cost : " << tc.first << std::endl;
    }
}