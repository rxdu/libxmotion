#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"
#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    // load map
    stopwatch::StopWatch timer;
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    // std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane.osm");
    // std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/short_segment.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }
    map->PrintInfo();
    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;
    RoadMapViz::SetupRoadMapViz(map, 10);
    // RoadMapViz::ShowLanes(true, 5, "test_lane", true);
    // for (auto &chn : map->traffic_map_->GetAllTrafficChannels())
    // {
    //     // RoadMapViz::ShowTrafficChannelCenterline(chn);
    //     chn->PrintInfo();
    //     RoadMapViz::ShowTrafficChannel(*chn.get(), 5);
    // }

    /****************************************************************************/

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(map);

    // discretize lane
    auto all_channels = traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    auto target_channel = all_channels[1];
    target_channel->DiscretizeChannel(5, 1.2, 5);

    RoadMapViz::ShowTrafficChannel(*target_channel.get());
    // RoadMapViz::ShowTrafficChannel(*target_channel.get(), 20, "horizontal_lane", true);

    /****************************************************************************/

    std::cout << "channel length: " << target_channel->center_curve_.GetTotalLength() << std::endl;

    // const int32_t s_size = static_cast<int32_t>(target_channel->center_curve_.GetTotalLength() / 5.0);
    MarkovOccupancy<20, 5> occupancy(0, 100, 0, 20);
    occupancy.SetupMarkovModel(33, 3 * 3, 5, 1 * 1);

    std::cout << "------------------------------" << std::endl;

    for (int i = 0; i < 12; i = i + 3)
        std::cout << "Occupancy at t_(" << i << ") : \n"
                  << occupancy.GetOccupancyDistribution(i) << "\n"
                  << std::endl;

    return 0;
}