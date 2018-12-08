#include <iostream>

// #include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    RoadMapViz::SetupRoadMapViz(loader.road_map, 3);

    auto ids = loader.road_map->FindOccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

    for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    {
        chn->PrintInfo();
        RoadMapViz::ShowTrafficChannel(*chn.get());
    }

    // RoadMapViz::ShowTrafficChannel(map->traffic_map_->GetAllTrafficChannels().front());

    return 0;
}