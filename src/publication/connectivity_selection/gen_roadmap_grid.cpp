#include <iostream>

#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    RoadMapViz::SetupRoadMapViz(map, 10);

    auto ids = map->OccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

    // for (auto &chn : map->traffic_map_->GetAllTrafficChannels())
    // {
    //     chn->PrintInfo();
    //     RoadMapViz::ShowTrafficChannel(*chn.get());
    // }

    RoadMapViz::ShowTrafficChannel(*map->traffic_map_->GetAllTrafficChannels()[4].get(), "roadmap_grid", true);

    return 0;
}