#include <iostream>

#include "threat_ranking/topogeo_graph.hpp"
#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");
    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    TopoGeoGraph graph;

    graph.GenerateGraph();

    std::cout << "topo-geo graph generated" << std::endl;

    // std::vector<std::string> lanelets = graph.BacktrackVertices(9);
    // LightViz::ShowMatrixAsColorMap(map->GetLaneDrivableGrid(lanelets)->GetGridMatrix(false), "driving_part", true);

    // auto boundary_matrix = map->GetFullLaneBoundaryGrid()->GetGridMatrix(true);
    // LightViz::ShowMatrixAsColorMap(boundary_matrix, "roadnetwork", true);

    std::vector<int32_t> left_path;
    left_path.push_back(4);
    left_path.push_back(7);
    left_path.push_back(1);
    std::vector<std::string> left_lanelets = graph.FindInteractingLanes(left_path);
    std::vector<Polygon> left_roi;
    for (auto ll : left_lanelets)
        left_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons());

    // auto left_matrix = map->GetLaneDrivableGrid(left_lanelets)->GetGridMatrix(false);
    // Eigen::MatrixXd left_vis_matrix = (boundary_matrix.array() > 0 || left_matrix.array() < 1).select(map->mask_zero_, map->mask_ones_);
    // LightViz::ShowMatrixAsColorMap(left_vis_matrix, "left turn", true);

    std::cout << "----------------" << std::endl;

    std::vector<int32_t> right_path;
    right_path.push_back(4);
    right_path.push_back(10);
    right_path.push_back(3);
    std::vector<std::string> right_lanelets = graph.FindInteractingLanes(right_path);

    std::vector<Polygon> right_roi;
    for (auto ll : right_lanelets)
        right_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons());
    // auto right_matrix = map->GetLaneDrivableGrid(right_lanelets)->GetGridMatrix(false);
    // Eigen::MatrixXd right_vis_matrix = (boundary_matrix.array() > 0 || right_matrix.array() < 1).select(map->mask_zero_, map->mask_ones_);
    // LightViz::ShowMatrixAsColorMap(right_vis_matrix, "right turn", true);

    return 0;
}