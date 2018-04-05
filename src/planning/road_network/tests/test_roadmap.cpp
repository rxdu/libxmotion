#include <iostream>

#include "road_network/road_map.hpp"
#include "lightviz/matrix_viz.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    // map.GenerateDenseGrids(15);
    std::cout << "generated grid in " << timer.toc() << " seconds" << std::endl;

    map.OccupiedLanelet(CartCooridnate(60, 60));

    // LightViz::ShowMatrixAsColorMap(map.GetFullGrid()->GetGridMatrix(true), "roadnetwork", true);

    // std::vector<std::string> lanelets;
    // lanelets.push_back("s4");
    // lanelets.push_back("icm2");
    // lanelets.push_back("s1");
    auto lanelets = map.FindShortestRoute("s4", "s3");
    LightViz::ShowMatrixAsColorMap(map.GetLaneBoundsGrid(lanelets)->GetGridMatrix(true), "driving", true);

    return 0;
}