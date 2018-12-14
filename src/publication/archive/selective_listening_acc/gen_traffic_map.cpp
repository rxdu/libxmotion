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
    traffic_map->DiscretizeTrafficRegions(1.2 * 4);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    int32_t index = 0;

    auto chns = traffic_map->GetAllTrafficChannels();
    // for (auto &chn : chns)
    //     RoadMapViz::ShowVehicleFootprints(chn.lane_blocks, 10, "traffic_channel_" + std::to_string(index++), true);

    index = 0;
    auto flows = traffic_map->GetAllTrafficFlows();
    // for (auto &flow : flows)
    // {
    //     auto blks = flow->GetAllLaneBlocks();
    //     RoadMapViz::ShowVehicleFootprints(blks, 10, "traffic_flow_" + std::to_string(index++), true);
    // }

    auto sample_blks = chns[0].lane_blocks;
    auto flow_blks = flows[2]->GetAllLaneBlocks();
    sample_blks.insert(sample_blks.end(), flow_blks.begin(), flow_blks.end());
    RoadMapViz::ShowVehicleFootprints(sample_blks, 10, "lane_channel_flow", true);

    return 0;
}