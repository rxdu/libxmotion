#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"

#include "local_planner/lattice_graph.hpp"
#include "navviz/navviz.hpp"

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

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(2, 0.74, 3);

    VehicleEstimation ego_veh({56.5, 36, 85.0 / 180.0 * M_PI}, 15);

    /****************************************************************************/

    stopwatch::StopWatch timer;
    std::vector<StateLattice> path;
    // auto graph = LatticeGraph::Search(path, ego_chn, {2, 0}, 8);
    auto graph = LatticeGraph::Search(path, ego_chn, ego_veh.GetPose(), 8);
    std::cout << "search finished in " << timer.toc() << " seconds" << std::endl;

    TrafficViz::ShowLatticePathInTrafficChannel(graph, path, ego_chn.get(), "lattice graph path", true);

    return 0;
}