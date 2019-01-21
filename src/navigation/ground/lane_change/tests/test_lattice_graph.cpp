#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "local_planner/lattice_graph.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace librav;

int main()
{
    // load map
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane.osm");
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/short_segment.osm");

    // UGVNavViz::ShowLanes(true, 5, "test_lane", true);
    // for (auto &chn : map->traffic_map_->GetAllTrafficChannels())
    // {
    //     // UGVNavViz::ShowTrafficChannelCenterline(chn);
    //     chn->PrintInfo();
    //     UGVNavViz::ShowTrafficChannel(*chn.get(), 5);
    // }

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    all_channels[1]->DiscretizeChannel(5, 1.2, 5);

    // UGVNavViz::ShowTrafficChannel(*all_channels[1].get());
    // UGVNavViz::ShowTrafficChannel(*all_channels[1].get(), 20, "horizontal_lane", true);

    stopwatch::StopWatch timer;
    auto graph = LatticeGraph::Construct(all_channels[1], {0, 0}, 9);
    std::cout << "graph constructed in " << timer.toc() << " seconds" << std::endl;

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    std::cout << "number of vertices: " << graph->GetTotalVertexNumber() << std::endl;

#ifdef ENABLE_VIZ
    // UGVNavViz::ShowStateLattice(loader.road_map, lattices);
    UGVNavViz::ShowLatticeInTrafficChannel(loader.road_map, lattices, all_channels[1].get(), "lattice graph", true);
#endif 

    return 0;
}