#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "threat_field/collision_threat.hpp"

#include "stopwatch/stopwatch.h"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    RoadMapViz::SetupRoadMapViz(loader.road_map);

    CovarMatrix2d covar;
    covar << 1, 0,
        0, 1;
    VehicleEstimation veh1({35, 51.2, -10 / 180.0 * M_PI}, 10);

    auto ids = loader.road_map->OccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

    // for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    // {
    //     chn->PrintInfo();
    //     RoadMapViz::ShowTrafficChannel(*chn.get());
    // }

    RoadMapViz::ShowVehicle(veh1.GetFootprint());

    return 0;
}