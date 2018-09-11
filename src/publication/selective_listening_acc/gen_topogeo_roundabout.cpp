#include <iostream>

#include "road_map/topogeo_graph.hpp"
#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/natick_roundabout_full.osm");

    auto left_path = map->FindShortestRouteName("s5", "s1");
    std::vector<std::string> left_lanelets = map->FindInteractingLanes(left_path);
    std::vector<Polygon> left_roi;
    for (auto ll : left_lanelets)
        left_roi.push_back(map->GetLanePolygon(ll));
    // LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 10, "left-turn-lanelets", true);
    LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 5);

    // std::cout << "----------------" << std::endl;

    // auto right_path = map->FindShortestRouteName("s4", "s3");
    // std::vector<std::string> right_lanelets = map->FindInteractingLanes(right_path);
    // std::vector<Polygon> right_roi;
    // for (auto ll : right_lanelets)
    //     right_roi.push_back(map->GetLanePolygon(ll));
    // LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons(), 10, "right-turn-lanelets", true);

    return 0;
}