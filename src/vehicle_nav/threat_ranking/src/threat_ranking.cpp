/* 
 * threat_ranking.cpp
 * 
 * Created on: Aug 08, 2018 00:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/threat_ranking.hpp"

#include <string>
#include <cstdint>

#include "lightviz/lightviz.hpp"
#include "vehicle_viz/vehicle_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

ThreatRanking::ThreatRanking(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    // create togo-geographical graph
    tg_graph_ = std::make_shared<TopoGeoGraph>();
    tg_graph_->GenerateGraph();

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
            // LightViz::ShowPathSegmentCollisionFieldWithRoadMap(path_[i].ToPolyline(), motion_model_->GeneratePredictedCollisionField(i), road_map_);
            LightViz::ShowPathCollisionFieldWithRoadMap(path_[i].ToPolyline(), full_path, motion_model_->GeneratePredictedCollisionField(i), road_map_);
        }
    }
}