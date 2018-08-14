#include <iostream>

#include "threat_ranking/topogeo_graph.hpp"
#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    TopoGeoGraph graph;

    graph.GenerateGraph();

    std::cout << "topo-geo graph generated" << std::endl;

    std::vector<int32_t> left_path;
    left_path.push_back(4);
    left_path.push_back(7);
    left_path.push_back(1);
    std::vector<std::string> left_lanelets = graph.FindInteractingLanes(left_path);
    std::vector<Polygon> left_roi;
    for (auto ll : left_lanelets)
        left_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(left_roi, map->GetAllLanePolygons(), 10, "left-turn-lanelets", true);

    std::cout << "----------------" << std::endl;

    std::vector<int32_t> right_path;
    right_path.push_back(4);
    right_path.push_back(10);
    right_path.push_back(3);
    std::vector<std::string> right_lanelets = graph.FindInteractingLanes(right_path);

    std::vector<Polygon> right_roi;
    for (auto ll : right_lanelets)
        right_roi.push_back(map->GetLanePolygon(ll));
    LightViz::ShowFilledPolygon(right_roi, map->GetAllLanePolygons(), 10, "right-turn-lanelets", true);

    return 0;
}