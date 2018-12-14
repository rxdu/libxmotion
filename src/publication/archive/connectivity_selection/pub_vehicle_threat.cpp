#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "threat_field/vehicle_threat.hpp"

#include "stopwatch/stopwatch.h"
#include "ugvnav_viz/ugvnav_viz.hpp"

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

    stopwatch::StopWatch timer;

    std::shared_ptr<VehicleThreat> ct1 = std::make_shared<VehicleThreat>(veh1, loader.traffic_map);

    // ct1->GetOccupancyDistributionAt(3);
    ct1->ComputeOccupancyDistribution(3);

    std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    std::cout << "------------- all calculation finished -------------" << std::endl;

    TrafficViz::ShowVehicleOccupancyDistribution(ct1, 3, "occupancy_estimation", true);
    TrafficViz::ShowVehicleCollisionThreat(ct1, 3, "vehicle_threat", true);

    return 0;
}