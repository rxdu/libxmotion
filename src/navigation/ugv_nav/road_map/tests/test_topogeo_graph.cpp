#include <iostream>

#include "road_map/details/topogeo_graph.hpp"
#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");
    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    // std::cout << "topo-geo graph generated" << std::endl;

    auto left_path = map->FindShortestRouteName("s4", "s1");
    std::vector<std::string> left_lanelets = map->FindConflictingLanes(left_path);
    std::vector<Polygon> left_roi;
    for (auto ll : left_lanelets)
        left_roi.push_back(map->GetLanePolygon(ll));
    // LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 10, "left turn", true);
    LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 10);

    std::cout << "----------------" << std::endl;

    auto right_path = map->FindShortestRouteName("s4", "s3");
    std::vector<std::string> right_lanelets = map->FindConflictingLanes(right_path);
    std::vector<Polygon> right_roi;
    for (auto ll : right_lanelets)
        right_roi.push_back(map->GetLanePolygon(ll));
    // LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons(), 10, "right turn", true);
    LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons(), 10);

    return 0;
}