#include <iostream>

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    // std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");
    // RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full_with_ref.osm");
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    RoadMapViz::SetupRoadMapViz(map);

    auto ids = map->OccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

    // LightViz::ShowPolygon(map->GetAllLanePolygons(), 10);
    // LightViz::ShowPolyline(map->GetAllLaneBoundPolylines(), 10);
    // LightViz::ShowLanePolylines(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines());
    // LightViz::ShowPolylinePosition(map->GetAllLaneCenterPolylines(), 10);

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(map);

    for (auto &chn : traffic_map->GetAllTrafficChannels())
    {
        // RoadMapViz::ShowTrafficChannelCenterline(chn);
        chn->PrintInfo();
        RoadMapViz::ShowTrafficChannel(*chn.get());
    }

    // RoadMapViz::ShowTrafficChannel(map->traffic_map_->GetAllTrafficChannels().front());

    // std::vector<Polygon> roi;
    // roi.push_back(map->GetLanePolygon("s1"));
    // LightViz::ShowFilledPolygon(roi, map->GetAllLanePolygons());

    return 0;
}