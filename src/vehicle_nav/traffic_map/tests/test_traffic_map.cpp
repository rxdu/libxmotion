#include <iostream>

#include "road_map/road_map.hpp"

#include "traffic_map/traffic_map.hpp"
#include "tree/algorithms/traversal.hpp"

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

    RoadMapViz::SetupRoadMapViz(map);

    timer.tic();

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(map);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    // double remainder = 0;
    // auto region = traffic_map->graph_->GetVertex(map->GetLaneletIDFromName("s1"))->state_;
    // auto fps = traffic_map->DecomposeTrafficRegion(region, remainder, 5.0);
    // RoadMapViz::ShowVehicleFootprints(fps, 10, "fp-decomp", false);

    auto fps = traffic_map->DiscretizeRoadNetwork(1.2 * 4);
    // RoadMapViz::ShowVehicleFootprints(fps, 10, "fp-decomp", true);

    // auto chns = traffic_map->FindConflictingChannels("s4", "s1");
    // auto chns = traffic_map->FindConflictingChannels("s4", "s3");
    // for(auto& chn : chns)
    //     std::cout << chn.source << " -> " << chn.sink << std::endl;

    // std::vector<std::string> lanelets;
    // lanelets.push_back("s4");
    // lanelets.push_back("icm2");
    // lanelets.push_back("s1");
    // auto fps = traffic_map->DecomposeCenterlines(lanelets, 3.0);

    // // RoadMapViz::ShowVehicleFootprints(fps, 10, "fp", true);

    // std::vector<std::string> lanelets2;
    // lanelets2.push_back("s2");
    // lanelets2.push_back("is1");
    // lanelets2.push_back("s1");
    // auto fps2 = traffic_map->DecomposeCenterlines(lanelets2, 3.0);

    // std::vector<Polygon> all_fps;
    // all_fps.insert(all_fps.end(), fps.begin(), fps.end());
    // all_fps.insert(all_fps.end(), fps2.begin(), fps2.end());
    // RoadMapViz::ShowVehicleFootprints(all_fps, 10, "fp", true);

    return 0;
}