#include <iostream>

#include "road_network/road_map.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm", 10);

    if (!map.MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    // map.GenerateDenseGrids(15);
    std::cout << "generated grid in " << timer.toc() << " seconds" << std::endl;

    // map.OccupiedLanelet(CartCooridnate(60, 60));

    // LightViz::ShowMatrixAsColorMap(map.GetFullLaneBoundaryGrid()->GetGridMatrix(true), "roadnetwork", true);
    LightViz::ShowMatrixAsColorMap(map.GetFullDrivableAreaGrid()->GetGridMatrix(true), "drivable", true);

    // std::vector<std::string> lanelets;
    // lanelets.push_back("s4");
    // lanelets.push_back("icm2");
    // lanelets.push_back("s1");
    auto lanelets = map.FindShortestRoute("s4", "s1");
    // LightViz::ShowMatrixAsColorMap(map.GetLaneBoundGrid(lanelets)->GetGridMatrix(true), "driving", true);

    // auto drivable = map.GetLaneletDrivableArea("s3");
    // auto drivable = map.GetFullDrivableAreaGrid();
    // LightViz::ShowMatrixAsColorMap(drivable->GetGridMatrix(true), "drivable", true);

    // auto matrix = map.GetFullLaneBoundaryGrid()->GetGridMatrix(false);
    // auto matrix = map.GetLaneBoundGrid(lanelets)->GetGridMatrix(false);
    // auto sgrid = GridDecomposer::CreateSquareGridFromMatrix(matrix, 5);
    // LightViz::ShowSquareGrid(sgrid.get(), 100, "Square Grid", true);

    // auto sgrid = map.GetLaneBoundGrid(lanelets)->ConvertToSquareGrid(5);
    // auto sgrid = std::make_shared<SquareGrid>(map.GetLaneBoundGrid(lanelets)->GetGridMatrix(false), 5);
    // LightViz::ShowSquareGrid(sgrid.get(), 100, "Square Grid", true);

    auto sgrid = SquareGrid(map.GetLaneBoundGrid(lanelets)->GetGridMatrix(false), 10);
    LightViz::ShowSquareGrid(&sgrid, 100, "Square Grid", true);

    return 0;
}