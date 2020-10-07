/* 
 * route_planner.hpp
 * 
 * Created on: Dec 09, 2018 07:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROUTE_PLANNER_HPP
#define ROUTE_PLANNER_HPP

#include <memory>

#include "cav_motion/reference_route.hpp"
#include "traffic_map/traffic_map.hpp"

namespace autodrive
{
class RoutePlanner
{
    struct RouteCandidate
    {
        RouteCandidate() = default;
        RouteCandidate(std::string ls, std::string lg, std::vector<std::string> ll) : lanelet_s(ls), lanelet_g(lg), route(ll) {}

        std::string lanelet_s;
        std::string lanelet_g;
        std::vector<std::string> route;
    };

  public:
    RoutePlanner(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap);

    bool SearchRoute(Position2d start, Position2d goal, ReferenceRoute *route);

  private:
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;
};
} // namespace autodrive

#endif /* ROUTE_PLANNER_HPP */
