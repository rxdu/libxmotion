#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "threat_field/vehicle_threat.hpp"

#include "stopwatch/stopwatch.h"
#include "navviz/navviz.hpp"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    TrafficViz::SetupTrafficViz(loader.road_map);

    //////////////////////////////////////////////////

    CovarMatrix2d pos_covar;
    pos_covar << 1, 0,
        0, 1;
    VehicleEstimation veh1({35, 51.2, -10 / 180.0 * M_PI}, 10);
    veh1.SetPositionVariance(pos_covar);
    veh1.SetSpeedVariance(2 * 2);

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels().back();
    // TrafficViz::ShowVehicleInChannel(veh1.GetFootprint(), *ego_chn.get());

    stopwatch::StopWatch timer;

    std::shared_ptr<VehicleThreat> ct1 = std::make_shared<VehicleThreat>(veh1, loader.traffic_map);

    ct1->ComputeOccupancyDistribution(5, true);

    std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    std::cout << "------------- all calculation finished -------------" << std::endl;

    // TrafficViz::ShowVehicleOccupancyDistribution(ct1, "occupancy_estimation");
    TrafficViz::ShowVehicleCollisionThreat(ct1, 4, "occupancy_estimation", false);
    // TrafficViz::ShowVehicleIntervalCollisionThreat(ct1, 4, "occupancy_estimation_interval", false);

    // for (int i = 0; i < 5; ++i)
    // TrafficViz::ShowVehicleCollisionThreat(ct1, i, "occupancy_estimation", false);

    return 0;
}