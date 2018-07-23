#include <iostream>
#include <memory>

#include "traffic_flow/road_cost_map.hpp"
#include "road_network/road_map.hpp"
#include "graph/algorithms/dijkstra_tr.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;
    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm", 10);
    std::cout << "generated grid in " << timer.toc() << " seconds" << std::endl;
    auto lanelets = map.FindShortestRoute("s4", "s1");
    // LightViz::ShowMatrixAsColorMap(map.GetLaneBoundGrid(lanelets)->GetGridMatrix(true), "driving", true);

    // auto drivable = map.GetLaneletDrivableArea("s3");
    // auto drivable = map.GetFullDrivableAreaGrid();
    // LightViz::ShowMatrixAsColorMap(drivable->GetGridMatrix(true), "drivable", true);

    auto matrix = map.GetFullLaneBoundaryGrid()->GetGridMatrix(false);
    // auto matrix = map.GetLaneBoundGrid(lanelets)->GetGridMatrix(false);
    auto sgrid = std::make_shared<RoadSquareGrid>(matrix, 10);
    LightViz::ShowSquareGrid(sgrid.get(), 100, "Square Grid", true);

    std::cout << "grid size: " << sgrid->SizeX() << " , " << sgrid->SizeY() << std::endl;
    auto nbs = sgrid->GetNeighbours(0, 0, true);
    for (auto &n : nbs)
    {
        std::cout << "n: " << n->id << std::endl;
        if (n->label == SquareCellLabel::FREE)
            std::cout << "free" << std::endl;
    }
    std::cout << "check neighbour: " << nbs.size() << std::endl;

    timer.tic();
    RoadSquareCell *tile_s = sgrid->GetCell(0, 2);
    auto graph = DijkstraTraversal::IncSearch(tile_s, GetNeighbourFunc_t<RoadSquareCell *>(GetRoadSquareGridNeighbour(sgrid)));
    std::cout << "traversal finished in " << timer.toc() << " seconds, vertex visited: " << graph.GetGraphVertexNumber() << std::endl;

    LightViz::ShowSquareGridGraph(sgrid.get(), &graph, 100);

    RoadCostMap::GenerateGraphCostMap(sgrid.get(), &graph);
    LightViz::ShowSquareGridGraphCost(sgrid.get());

    return 0;
}