#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "mission/cav_motion_manager.hpp"

#include "stopwatch/stopwatch.h"

using namespace ivnav;

int main()
{
    CAVMotionManager manager("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    if (!manager.IsReady())
    {
        std::cerr << "ERROR: planner is not set up properly" << std::endl;
        return -1;
    }

    manager.Run();

    return 0;
}