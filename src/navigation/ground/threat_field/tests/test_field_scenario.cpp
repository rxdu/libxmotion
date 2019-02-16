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

    // // ------------------- vehicle 1 ---------------------- //

    // CovarMatrix2d pos_covar1;
    // pos_covar1 << 2, 0,
    //     0, 2;
    // auto ego_chn1 = loader.traffic_map->GetAllTrafficChannels()[4];

    // VehicleEstimation veh1({35, 59, -7 / 180.0 * M_PI}, 10);
    // veh1.SetPositionVariance(pos_covar1);
    // veh1.SetSpeedVariance(3 * 3);

    // // ------------------- vehicle 2 ---------------------- //

    // CovarMatrix2d pos_covar2;
    // pos_covar2 << 1, 0,
    //     0, 1;
    // auto ego_chn2 = loader.traffic_map->GetAllTrafficChannels()[4];

    // VehicleEstimation veh2({89, 52, -7 / 180.0 * M_PI}, 10);
    // veh2.SetPositionVariance(pos_covar2);
    // veh2.SetSpeedVariance(2 * 2);

    // // ------------------- vehicle 3 ---------------------- //

    // CovarMatrix2d pos_covar3;
    // pos_covar3 << 0.25, 0,
    //     0, 0.25;
    // auto ego_chn3 = loader.traffic_map->GetAllTrafficChannels()[0];

    // VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, 10);
    // veh3.SetPositionVariance(pos_covar3);
    // veh3.SetSpeedVariance(1 * 1);

    // // ------------------- vehicle 4 ---------------------- //

    // CovarMatrix2d pos_covar4;
    // pos_covar4 << 1, 0,
    //     0, 1;
    // auto ego_chn4 = loader.traffic_map->GetAllTrafficChannels()[5];

    // VehicleEstimation veh4({52, 35, -95 / 180.0 * M_PI}, 10);
    // veh4.SetPositionVariance(pos_covar4);
    // veh4.SetSpeedVariance(2 * 2);

    // // ------------------- vehicle 5 ---------------------- //

    // CovarMatrix2d pos_covar5;
    // pos_covar5 << 1, 0,
    //     0, 1;
    // auto ego_chn5 = loader.traffic_map->GetAllTrafficChannels()[0];

    // VehicleEstimation veh5({40, 64, 171 / 180.0 * M_PI}, 10);
    // veh5.SetPositionVariance(pos_covar5);
    // veh5.SetSpeedVariance(2 * 2);

    // std::vector<Polygon> vehs = {veh1.GetFootprint(), veh2.GetFootprint(), veh3.GetFootprint(), veh4.GetFootprint(), veh5.GetFootprint()};

    // #ifdef ENABLE_VIZ
    // UGVNavViz::ShowVehicleOnMap(loader.road_map, vehs, "vehicle_config", true);

    // for(auto& veh:vehs)
    //     UGVNavViz::ShowVehicleInChannel(veh, *ego_chn1.get());

    // for(auto chn : loader.traffic_map->GetAllTrafficChannels())
    //     UGVNavViz::ShowVehicleInChannel(veh1.GetFootprint(), *chn.get());
    // #endif

#ifdef ENABLE_VIZ
    UGVNavViz::ShowLanes(loader.road_map);

    for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    {
        chn->PrintInfo();
        chn->DiscretizeChannel(5, 0.74, 5);
        UGVNavViz::ShowTrafficChannel(loader.road_map, chn.get());
    }

    // UGVNavViz::ShowTrafficChannel(loader.road_map, loader.traffic_map->GetAllTrafficChannels().front().get());
#endif

    return 0;
}