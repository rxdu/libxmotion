#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

#include "lattice_planner/lattice_graph.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

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
    all_channels[1]->DiscretizeChannel(5, 1.2, 5);

    // RoadMapViz::ShowTrafficChannel(*all_channels[1].get());
    // RoadMapViz::ShowTrafficChannel(*all_channels[1].get(), 20, "horizontal_lane", true);

    timer.tic();
    auto graph = LatticeGraph::Construct(all_channels[1], {0, 0}, 9);
    std::cout << "graph constructed in " << timer.toc() << " seconds" << std::endl;

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    std::cout << "number of vertices: " << graph->GetGraphVertexNumber() << std::endl;

    // LightViz::ShowStateLattice(lattices);
    // RoadMapViz::ShowLatticeInTrafficChannel(lattices, *all_channels[1].get(), "lattice graph", true);

    return 0;
}