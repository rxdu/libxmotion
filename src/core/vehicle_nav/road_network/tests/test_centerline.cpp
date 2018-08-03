#include <iostream>

#include "road_network/road_map.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm", 10);

    if (!map.MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    LightViz::ShowMatrixAsColorMap(map.GetFullLaneBoundaryGrid()->GetGridMatrix(true), "roadnetwork", true);

    return 0;
}