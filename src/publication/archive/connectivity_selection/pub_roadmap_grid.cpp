#include <iostream>

#include "traffic_map/map_loader.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    RoadMapViz::SetupRoadMapViz(loader.road_map, 10);

    RoadMapViz::ShowTrafficChannel(loader.traffic_map->GetAllTrafficChannels()[4].get(), "roadmap_grid", true);

    return 0;
}