#include <iostream>
#include <cstdint>
#include <cmath>

#include "traffic_map/map_loader.hpp"
#include "prediction/dynamic_threat_model.hpp"

#include "stopwatch/stopwatch.h"
#include "lightviz/roadmap_viz.hpp"
#include "lightviz/traffic_viz.hpp"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");

    //////////////////////////////////////////////////

    CovarMatrix2d pos_covar;
    pos_covar << 1, 0,
        0, 1;
    VehicleEstimation veh1({35, 51.2, -10 / 180.0 * M_PI}, 10);
    veh1.SetPositionVariance(pos_covar);
    veh1.SetSpeedVariance(2 * 2);

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels().back();

    VehicleStaticThreat static_threat(veh1.GetPose(), 1);

    auto canvas = UGVNavViz::CreateCanvas(loader.road_map);
    UGVNavViz::ShowVehicleInChannel(loader.road_map, veh1.GetFootprint(), ego_chn.get());
    UGVNavViz::ShowVehicleStaticThreat(loader.road_map, static_threat);

    return 0;
}