/* 
 * route_planner.cpp
 * 
 * Created on: Dec 09, 2018 07:14
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cav_motion/route_planner.hpp"

#include <limits>

namespace librav
{
RoutePlanner::RoutePlanner(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap) : road_map_(rmap),
                                                                                              traffic_map_(tmap)
{
}

bool RoutePlanner::SearchRoute(Position2d start, Position2d goal, ReferenceRoute *route)
{
    std::vector<int32_t> start_lanelets = road_map_->FindOccupiedLanelet({start.x, start.y});
    std::vector<int32_t> goal_lanelets = road_map_->FindOccupiedLanelet({goal.x, goal.y});

    if (start_lanelets.empty() || goal_lanelets.empty())
        return false;

    std::vector<RouteCandidate> route_candidates;
    for (auto &sll : start_lanelets)
    {
        for (auto &gll : goal_lanelets)
        {
            std::vector<std::string> route = road_map_->FindShortestRouteName(sll, gll);

            if (!route.empty())
                route_candidates.push_back({road_map_->GetLaneletNameFromID(sll), road_map_->GetLaneletNameFromID(gll), route});
        }
    }

    std::cout << "number of route candidates found: " << route_candidates.size() << std::endl;

    if (route_candidates.empty())
    {
        return false;
    }
    else if (route_candidates.size() == 1)
    {
        auto candidate = route_candidates.front();
        *route = ReferenceRoute(road_map_, start, goal, candidate.lanelet_s, candidate.lanelet_g, candidate.route);
    }
    else
    {
        std::vector<ReferenceRoute> routes;
        for (auto &candidate : route_candidates)
            routes.push_back(ReferenceRoute(road_map_, start, goal, candidate.lanelet_s, candidate.lanelet_g, candidate.route));
        int min_idx = 0;
        double min_length = std::numeric_limits<double>::max();
        for (int i = 0; i < routes.size(); ++i)
        {
            auto rt = routes[i];
            if (rt.GetLength() < min_length)
            {
                min_idx = i;
                min_length = rt.GetLength();
            }
        }
        *route = routes[min_idx];
    }

    return true;
}

} // namespace librav