#include <iostream>

#include "traffic_map/map_loader.hpp"

#include "lightviz/lightviz.hpp"
#include "navviz/navviz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    RoadMapViz::SetupRoadMapViz(loader.road_map, 10);

    CovarMatrix2d pos_covar;
    pos_covar << 1, 0,
        0, 1;
    VehicleEstimation veh1({89, 52, -7 / 180.0 * M_PI}, 10);
    veh1.SetPositionVariance(pos_covar);
    veh1.SetSpeedVariance(2 * 2);

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[3];
    // TrafficViz::ShowVehicleInChannel(veh1.GetFootprint(), *ego_chn.get());

    stopwatch::StopWatch timer;

    std::shared_ptr<VehicleThreat> ct1 = std::make_shared<VehicleThreat>(veh1, loader.traffic_map);

    // ct1->GetOccupancyDistributionAt(2);
    ct1->ComputeOccupancyDistribution(2);

    std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    std::cout << "------------- all calculation finished -------------" << std::endl;

    TrafficViz::ShowVehicleOccupancyDistribution(ct1, 2, "lane_occupancy", true);

    return 0;
}