#include <iostream>

#include "road_map/topogeo_graph.hpp"
#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");
    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    // TopoGeoGraph graph(map.get());

    // graph.ConstructGraph();
    // graph.GenerateGraph();

    // std::cout << "topo-geo graph generated" << std::endl;

    // std::vector<std::string> lanelets = graph.BacktrackVertices(9);
    // LightViz::ShowMatrixAsColorMap(map->GetLaneDrivableGrid(lanelets)->GetGridMatrix(false), "driving_part", true);

    // auto boundary_matrix = map->GetFullLaneBoundaryGrid()->GetGridMatrix(true);
    // LightViz::ShowMatrixAsColorMap(boundary_matrix, "roadnetwork", true);

    auto left_path = map->FindShortestRouteName("s4", "s1");
    std::vector<std::string> left_lanelets = map->FindInteractingLanes(left_path);
    std::vector<Polygon> left_roi;
    for (auto ll : left_lanelets)
        left_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 10, "left turn", true);

    std::cout << "----------------" << std::endl;

    auto right_path = map->FindShortestRouteName("s4", "s3");
    std::vector<std::string> right_lanelets = map->FindInteractingLanes(right_path);
    std::vector<Polygon> right_roi;
    for (auto ll : right_lanelets)
        right_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons(), 10, "right turn", true);

    return 0;
}