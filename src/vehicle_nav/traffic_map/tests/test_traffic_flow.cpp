#include <iostream>

#include "road_map/road_map.hpp"

#include "traffic_map/traffic_map.hpp"

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

    /********** create traffic map **********/

    timer.tic();

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(map);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////

    auto fps = traffic_map->DiscretizeRoadNetwork(1.2 * 4);
    // RoadMapViz::ShowVehicleFootprints(fps, 10, "fp-decomp", true);

    auto cflows = traffic_map->GetConflictingFlows("s4", "s1");
    std::cout << "number of conflicting cflows: " << cflows.size() << std::endl;
    for (auto &tf : cflows)
    {
        auto blks = tf.GetAllLaneBlocks();
        std::cout << " - number of blocks: " << blks.size() << std::endl;
        RoadMapViz::ShowVehicleFootprints(blks, 10);
    }

    auto ego_chn = traffic_map->GetTrafficChannel("s4", "s1");
    auto ego_flow = TrafficFlow(ego_chn);

    ego_flow.CheckConflicts(cflows);

    return 0;
}