#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "prediction/vehicle_threat.hpp"

#include "stopwatch/stopwatch.h"
// #include "lightviz/navviz.hpp"

using namespace autodrive;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, pos_covar3, 10, 1 * 1);

    VehicleThreat vthreat(veh3, loader.traffic_map);

    stopwatch::StopWatch timer;
    vthreat.ComputeOccupancyDistribution(5);
    std::cout << "propagation finished in " << timer.toc() << " seconds." << std::endl;   

    vthreat.PrintThreatRecordInfo();
}