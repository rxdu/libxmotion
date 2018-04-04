#include <iostream>

#include "road_network/road_map.hpp"
#include "lightviz/matrix_viz.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    stopwatch::StopWatch timer;
    map.GenerateDenseGrid(5);
    std::cout <<"generated grid in " << timer.toc() << " seconds" << std::endl;

    LightViz::ShowMatrixAsColorMap(map.dense_grid_->GetGridMatrix(true), "roadnetwork", true);

    return 0;
}