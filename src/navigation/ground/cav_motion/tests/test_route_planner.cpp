#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "cav_motion/route_planner.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    auto start_point = loader.traffic_map->FindTrafficChannel("s1", "s2")->ConvertToGlobalCoordinate({150, 0});
    auto goal_point = loader.traffic_map->FindTrafficChannel("s1", "s8")->ConvertToGlobalCoordinate({250, 0});

    // VehicleState start_state(-1, {start_point.x, start_point.y, start_point.theta}, 0);
    // VehicleState goal_state(-1, {goal_point.x, goal_point.y, goal_point.theta}, 0);

    std::cout << "start position: " << start_point.x << " , " << start_point.y << std::endl;
    std::cout << "goal position: " << goal_point.x << " , " << goal_point.y << std::endl;

    RoutePlanner rplanner(loader.road_map, loader.traffic_map);

    ReferenceRoute route;
    bool res = rplanner.SearchRoute({start_point.x, start_point.y}, {goal_point.x, goal_point.y}, &route);
    route.PrintInfo();

    RoadMapViz::SetupRoadMapViz(loader.road_map, 3);
    RoadMapViz::ShowTrafficChannel(&route);

    return 0;
}