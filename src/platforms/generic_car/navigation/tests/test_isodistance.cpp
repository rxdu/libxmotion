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

    // for (auto &channel : tf.GetTrafficChannelSources())
    // {
    //     Eigen::MatrixXd matrix = tf.GetTrafficChannelMatrix(channel);
    //     LightViz::ShowMatrixAsColorMap(matrix, channel, true);
    // }

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

    return 0;
}