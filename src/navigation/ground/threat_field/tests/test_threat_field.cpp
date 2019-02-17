#include <iostream>

// #include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/one_way_merging_horizontal.osm");
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    // auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[4];

    // ------------------- vehicle 1 ---------------------- //

    CovarMatrix2d pos_covar1;
    pos_covar1 << 2, 0,
        0, 2;
    VehicleEstimation veh1({15, 11, 0 / 180.0 * M_PI}, 10);
    veh1.SetPositionVariance(pos_covar1);
    veh1.SetSpeedVariance(3 * 3);

    // ------------------- vehicle 2 ---------------------- //

    CovarMatrix2d pos_covar2;
    pos_covar2 << 1, 0,
        0, 1;
    VehicleEstimation veh2({35, 11, 0 / 180.0 * M_PI}, 10);
    veh2.SetPositionVariance(pos_covar2);
    veh2.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 3 ---------------------- //

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    VehicleEstimation veh3({85, 11, 0 / 180.0 * M_PI}, 10);
    veh3.SetPositionVariance(pos_covar3);
    veh3.SetSpeedVariance(1 * 1);

    // ------------------- vehicle 4 ---------------------- //

    CovarMatrix2d pos_covar4;
    pos_covar4 << 1, 0,
        0, 1;
    VehicleEstimation veh4({25, 15.5, 0 / 180.0 * M_PI}, 10);
    veh4.SetPositionVariance(pos_covar4);
    veh4.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 5 ---------------------- //

    CovarMatrix2d pos_covar5;
    pos_covar5 << 1, 0,
        0, 1;
    VehicleEstimation veh5({55, 15.5, 0 / 180.0 * M_PI}, 10);
    veh5.SetPositionVariance(pos_covar5);
    veh5.SetSpeedVariance(2 * 2);

    std::vector<Polygon> vehs = {veh1.GetFootprint(), veh2.GetFootprint(), veh3.GetFootprint(), veh4.GetFootprint(), veh5.GetFootprint()};

#ifdef ENABLE_VIZ
    UGVNavViz::ShowVehicleOnMap(loader.road_map, vehs, "vehicle_config", false);

    // UGVNavViz::ShowLanes(loader.road_map);

    // for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    // {
    //     chn->PrintInfo();
    //     chn->DiscretizeChannel(5, 0.74, 5);
    //     UGVNavViz::ShowTrafficChannel(loader.road_map, chn.get());
    // }
#endif

    return 0;
}