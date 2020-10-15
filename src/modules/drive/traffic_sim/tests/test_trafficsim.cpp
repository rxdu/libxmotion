#include <iostream>

#include "traffic_map/map_loader.hpp"
#include "traffic_sim/traffic_sim.hpp"
#include "stopwatch/stopwatch.h"

using namespace ivnav;

#define LOOP_PERIOD 500

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    std::cout << "started traffic sim" << std::endl;
    TrafficSim sim(map);

    sim.SetDuration(180);
    sim.SetStartTime(0);
    sim.SetStepSize(1);

    // simulation loop
    while (true)
    {
        sim.UpdateTraffic(LOOP_PERIOD);
    }

    return 0;
}