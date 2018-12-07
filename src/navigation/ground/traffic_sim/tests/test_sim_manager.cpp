#include <iostream>

#include "traffic_sim/traffic_sim_manager.hpp"
#include "traffic_sim/sim_scenarios.hpp"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
    // TrafficSimConfig config;

    // config.tf = 15.0;
    // config.dt = 0.1;

    // config.map = "/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm";

    // VehicleInfo vinfo00(std::make_pair("s2", "s1"), 20.0);
    // vinfo00.init_speed = 15.0;

    // VehicleInfo vinfo01(std::make_pair("s2", "s1"), 40.0);
    // vinfo01.init_speed = 15.5;

    // VehicleInfo vinfo02(std::make_pair("s2", "s1"), 90.0);
    // vinfo02.init_speed = 15.5;

    // VehicleInfo vinfo10(std::make_pair("s6", "s3"), 15.0);
    // vinfo10.init_speed = 15.0;

    // VehicleInfo vinfo11(std::make_pair("s6", "s3"), 35.0);
    // vinfo11.init_speed = 15.0;

    // VehicleInfo vinfo12(std::make_pair("s6", "s3"), 85.0);
    // vinfo12.init_speed = 16.0;

    // config.surrounding_vehicles.push_back(vinfo00);
    // config.surrounding_vehicles.push_back(vinfo01);
    // config.surrounding_vehicles.push_back(vinfo02);
    // config.surrounding_vehicles.push_back(vinfo10);
    // config.surrounding_vehicles.push_back(vinfo11);
    // config.surrounding_vehicles.push_back(vinfo12);

    TrafficSimConfig config = SimScenario::GenerateScenarioCase1();

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