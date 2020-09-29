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
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    auto ids = loader.road_map->FindOccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

#ifdef ENABLE_VIZ
    for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    {
        chn->PrintInfo();
        chn->DiscretizeChannel(5, 0.74, 5);
        UGVNavViz::ShowTrafficChannel(loader.road_map, chn.get());
    }

    UGVNavViz::ShowTrafficChannel(loader.road_map, loader.traffic_map->GetAllTrafficChannels().front().get());
#endif

    return 0;
}