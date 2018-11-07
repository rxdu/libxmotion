#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"

#include "lattice_planner/lattice_graph.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    // load map
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    TrafficViz::SetupTrafficViz(loader.road_map, 10);

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(10, 0.74, 1);

    /****************************************************************************/

    // stopwatch::StopWatch timer;
    // auto graph = LatticeGraph::Construct(ego_chn, {0, 0}, 9);
    // std::cout << "graph constructed in " << timer.toc() << " seconds" << std::endl;
    // std::vector<StateLattice> lattices;
    // for (auto &edge : graph->GetAllEdges())
    //     lattices.push_back(edge->cost_);
    // std::cout << "number of vertices: " << graph->GetGraphVertexNumber() << std::endl;

    // TrafficViz::ShowLatticeInTrafficChannel(lattices, *ego_chn.get(), "lattice graph", true);

    /****************************************************************************/

    auto path = LatticeGraph::Search(ego_chn, {2, 0}, 8);

    TrafficViz::ShowLatticePathInTrafficChannel(path, *ego_chn.get(), "lattice graph path", false);

    return 0;
}