#include <iostream>

#include "road_map/road_map.hpp"

#include "threat_ranking/traffic_map.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    /********** create road map **********/
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    timer.tic();

    std::shared_ptr<TrafficMap> tflow_map = std::make_shared<TrafficMap>(map);

    std::vector<std::string> lanelets;
    lanelets.push_back("s4");
    lanelets.push_back("icm2");
    lanelets.push_back("s1");
    // lanelets.push_back("s6");
    // lanelets.push_back("im2");
    // lanelets.push_back("s5");
    auto fps = tflow_map->DecomposeCenterlines(lanelets,1.0);

    RoadMapViz::SetupRoadMapViz(map);

    RoadMapViz::ShowVehicleFootprints(fps, 10 , "fp", true);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    return 0;
}