#include <iostream>
#include <memory>

#include "navigation/map_analysis.hpp"
#include "road_network/road_map.hpp"
#include "navigation/traffic_flow_map.hpp"
#include "graph/algorithms/dijkstra_tr.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;
    // std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full_with_ref2.osm", 10);
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm", 10);
    std::cout << "loaded map in " << timer.toc() << " seconds" << std::endl;

    LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    TrafficFlowMap tf(map);
    tf.BuildRoadGrid(10);
    LightViz::ShowSquareGrid(tf.road_grid_.get(), 100, "raw grid", true);

    tf.AddTrafficFlowSource("s2", GridCoordinate(120, 18));
    tf.AddTrafficFlowSource("s4", GridCoordinate(51, 67));
    tf.AddTrafficFlowSource("s6", GridCoordinate(4, 7));

    tf.AddTrafficFlowSink("s1");
    tf.AddTrafficFlowSink("s3");
    tf.AddTrafficFlowSink("s5");

    tf.IdentifyTrafficFlow();

    for (auto &channel : tf.GetTrafficChannelSources())
    {
        Eigen::MatrixXd matrix = tf.GetTrafficChannelMatrix(channel);
        LightViz::ShowMatrixAsColorMap(matrix, channel, true);
    }

    std::cout << "generating traffic flow map" << std::endl;
    tf.GenerateTrafficFlowMap();

    MapAnalysis::GenerateGraphCostMap(tf.road_grid_.get(), "s2");
    LightViz::ShowSquareGridGraphCost(tf.road_grid_.get(), 100, "distance-s2", true);

    MapAnalysis::GenerateGraphCostMap(tf.road_grid_.get(), "s4");
    LightViz::ShowSquareGridGraphCost(tf.road_grid_.get(), 100, "distance-s4", true);

    MapAnalysis::GenerateGraphCostMap(tf.road_grid_.get(), "s6");
    LightViz::ShowSquareGridGraphCost(tf.road_grid_.get(), 100, "distance-s6", true);

    MapAnalysis::GenerateTrafficDensityCostMap(tf.road_grid_.get());
    LightViz::ShowSquareGridGraphCost(tf.road_grid_.get(), 100, "traffic density", true);

    // std::vector<std::string> lanelets;
    // lanelets.push_back("s4");
    // lanelets.push_back("icm2");
    // lanelets.push_back("s1");
    // lanelets.push_back("im1");
    // lanelets.push_back("s3");
    // LightViz::ShowMatrixAsColorMap(map->GetLaneDrivableGrid(lanelets)->GetGridMatrix(false), "driving_part", true);
    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(false), "driving_full", true);

    /////////////////////////////////////////////////////////////////////////////////

    // // route planning
    // auto lanelets = map->FindShortestRoute("s4", "s1");
    // auto lanelets2 = map->FindShortestRoute("s4", "s3");
    // Eigen::MatrixXd s4_matrix = map->GetLaneBoundGrid(lanelets)->GetGridMatrix(false) + map->GetLaneBoundGrid(lanelets2)->GetGridMatrix(false) + map->GetFullDrivableAreaGrid()->GetGridMatrix(false);
    // LightViz::ShowMatrixAsColorMap(s4_matrix, "driving", true);

    // // create square grid
    // // auto matrix = map->GetFullLaneBoundaryGrid()->GetGridMatrix(false);
    // // Eigen::MatrixXd matrix = map->GetFullLaneBoundaryGrid()->GetGridMatrix(false) + map->GetFullDrivableAreaGrid()->GetGridMatrix(false);
    // // Eigen::MatrixXd matrix = map->GetLaneBoundGrid(lanelets)->GetGridMatrix(false) + map->GetFullDrivableAreaGrid()->GetGridMatrix(false);

    // std::vector<std::string> test_lanelets;
    // test_lanelets.push_back("s4");
    // test_lanelets.push_back("icm2");
    // test_lanelets.push_back("s1");
    // test_lanelets.push_back("im1");
    // test_lanelets.push_back("s3");
    // Eigen::MatrixXd matrix = map->GetLaneDrivableGrid(test_lanelets)->GetGridMatrix(false);
    // auto sgrid = std::make_shared<RoadSquareGrid>(matrix, 10);
    // std::cout << "square grid size: " << sgrid->SizeX() << " , " << sgrid->SizeY() << std::endl;
    // LightViz::ShowSquareGrid(sgrid.get(), 100, "Square Grid", true);

    // timer.tic();
    // // RoadSquareCell *tile_s = sgrid->GetCell(6, 0);
    // RoadSquareCell *tile_s = sgrid->GetCell(51, 67);
    // auto graph = DijkstraTraversal::IncSearch(tile_s, GetNeighbourFunc_t<RoadSquareCell *>(GetSquareGridNeighbour(sgrid)));
    // std::cout << "traversal finished in " << timer.toc() << " seconds, vertex visited: " << graph.GetGraphVertexNumber() << std::endl;

    // // LightViz::ShowSquareGridGraph(sgrid.get(), &graph, 100);

    // // traffic flow analysis
    // MapAnalysis::GenerateGraphCostMap(sgrid.get(), &graph);
    // LightViz::ShowSquareGridGraphCost(sgrid.get(), &graph, 100, "distance", true);

    return 0;
}