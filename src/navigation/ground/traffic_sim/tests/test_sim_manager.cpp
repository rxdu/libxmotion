#include <iostream>

#include "traffic_sim/traffic_sim_manager.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
    TrafficSimConfig config;

    config.tf = 15.0;
    config.dt = 0.1;

    config.map = "/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm";

    VehicleInfo vinfo0(std::make_pair("s2", "s1"), 2.0);
    vinfo0.init_speed = 1.0;

    config.surrounding_vehicles.push_back(vinfo0);

    /*----------------------------------------------------------------*/

    TrafficSimManager sim_manager(config);

    if (!sim_manager.ValidateSimConfig())
    {
        std::cerr << "ERROR: invalid simulation config!" << std::endl;
        return -1;
    }

    sim_manager.RunSim();

    return 0;
}