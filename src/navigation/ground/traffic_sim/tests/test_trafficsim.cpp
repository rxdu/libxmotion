#include <iostream>

#include "traffic_sim/traffic_sim.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

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