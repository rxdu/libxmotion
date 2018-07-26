#include <iostream>

#include "traffic_flow/topogeo_graph.hpp"
#include "road_network/road_map.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm", 10);
    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    TopoGeoGraph graph;

    graph.GenerateGraph();

    std::cout << "topo-geo graph generated" << std::endl;

    // std::vector<std::string> lanelets = graph.BacktrackVertices(9);
    // LightViz::ShowMatrixAsColorMap(map->GetLaneDrivableGrid(lanelets)->GetGridMatrix(false), "driving_part", true);

    auto boundary_matrix = map->GetFullLaneBoundaryGrid()->GetGridMatrix(true);
    // LightViz::ShowMatrixAsColorMap(boundary_matrix, "roadnetwork", true);

    std::vector<int32_t> left_path;
    left_path.push_back(4);
    left_path.push_back(7);
    left_path.push_back(1);
    std::vector<std::string> left_lanelets = graph.FindInteractingLanes(left_path);
    auto left_matrix = map->GetLaneDrivableGrid(left_lanelets)->GetGridMatrix(false);
    Eigen::MatrixXd left_vis_matrix = (boundary_matrix.array() > 0 || left_matrix.array() < 1).select(map->mask_zero_, map->mask_ones_);
    LightViz::ShowMatrixAsColorMap(left_vis_matrix, "left turn", true);

    std::cout << "----------------" << std::endl;

    std::vector<int32_t> right_path;
    right_path.push_back(4);
    right_path.push_back(10);
    right_path.push_back(3);
    std::vector<std::string> right_lanelets = graph.FindInteractingLanes(right_path);
    auto right_matrix = map->GetLaneDrivableGrid(right_lanelets)->GetGridMatrix(false);
    Eigen::MatrixXd right_vis_matrix = (boundary_matrix.array() > 0 || right_matrix.array() < 1).select(map->mask_zero_, map->mask_ones_);
    LightViz::ShowMatrixAsColorMap(right_vis_matrix, "right turn", true);

    return 0;
}