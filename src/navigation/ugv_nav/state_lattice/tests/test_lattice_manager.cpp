#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "state_lattice/primitive_generator.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    RoadMapViz::SetupRoadMapViz(map);

    RoadMapViz::ShowLanes(true, 5, "test_lane", true);

    return 0;
}