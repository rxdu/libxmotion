/* 
 * threat_projector.cpp
 * 
 * Created on: Aug 02, 2018 21:58
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/threat_projector.hpp"

using namespace librav;

ThreatProjector::ThreatProjector(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    if (!road_map_->MapReady())
        std::cerr << "map didn't load correctly" << std::endl;
}

void ThreatProjector::SetRouteStartGoal(std::string start, std::string goal)
{
    start_lanelet_ = start;
    goal_lanelet_ = goal;

    route_ = road_map_->FindShortestRouteName(start_lanelet_, goal_lanelet_);
}

Eigen::MatrixXd ThreatProjector::GetRouteDrivableMask(bool normalize)
{
    return road_map_->GetLaneDrivableGrid(route_)->GetGridMatrix(normalize);
}