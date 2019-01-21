#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"

#include "local_planner/lattice_graph.hpp"
#include "local_planner/reference_trajectory.hpp"
#include "local_planner/lookahead_zone.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace librav;

int main()
{
    // load map
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm"); 

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();

    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(2, 0.74, 3);

    /****************************************************************************/

    stopwatch::StopWatch timer;
    std::vector<StateLattice> path;
    auto graph = LatticeGraph::Search(path, ego_chn, {2, 0}, 8);
    std::cout << "search finished in " << timer.toc() << " seconds" << std::endl;

    // UGVNavViz::ShowLatticePathInTrafficChannel(graph, path, *ego_chn.get(), "lattice graph path", true);

    ReferenceTrajectory traj(path);
    traj.GenerateConstantSpeedProfile(15);

    LookaheadZone zone(traj);

#ifdef ENABLE_VIZ
    UGVNavViz::ShowLatticePathWithLookaheadZone(loader.road_map, path, zone, ego_chn.get(), "path with lookahead");
#endif

    return 0;
}