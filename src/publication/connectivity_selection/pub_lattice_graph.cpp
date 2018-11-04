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
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");

    TrafficViz::SetupTrafficViz(loader.road_map, 15);

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    all_channels[1]->DiscretizeChannel(18, 1.2, 5);

    stopwatch::StopWatch timer;
    auto graph = LatticeGraph::Construct(all_channels[1], {0, 0}, 9);
    std::cout << "graph constructed in " << timer.toc() << " seconds" << std::endl;

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    std::cout << "number of vertices: " << graph->GetGraphVertexNumber() << std::endl;

    TrafficViz::ShowLatticeInTrafficChannel(lattices, *all_channels[1].get(), "lattice graph", true);

    return 0;
}