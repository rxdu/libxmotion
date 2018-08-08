/* 
 * threat_ranking.cpp
 * 
 * Created on: Aug 08, 2018 00:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/threat_ranking.hpp"

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
    // lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.uniform.data");
    lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.three-level.data");
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
    // ego_path_id_ = road_map_->FindShortestRoute(start, goal);
    ego_path_ = road_map_->FindShortestRouteName(start, goal);

    std::vector<int32_t> laneblock_ids;
    for (auto &ll_name : ego_path_)
        laneblock_ids.push_back(tg_graph_->ll_id_lookup_[ll_name]);
    active_lanelets_ = tg_graph_->FindInteractingLanes(laneblock_ids);
    drivable_mask_ = road_map_->GetLaneDrivableGrid(ego_path_);

    planner_->SetDrivableAreaMask(drivable_mask_);
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
    // auto path = planner.Search(start, goal);

    stopwatch::StopWatch timer;
    path_ = planner_->AStarSearch(start, goal);
    std::cout << "finished in " << timer.mtoc() << " seconds" << std::endl;
    lattice_manager_->SavePrimitivesToFile(path_, "path");
}