#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "cav_motion/cav_motion_manager.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    CAVMotionManager manager;

    if (!manager.IsReady())
    {
        std::cerr << "ERROR: planner is not set up properly" << std::endl;
        return -1;
    }

    manager.Run();

    return 0;
}