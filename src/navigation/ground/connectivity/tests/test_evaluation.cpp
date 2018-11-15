#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "connectivity/threat_evaluation.hpp"

#include "local_planner/lattice_graph.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    // load map
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    TrafficViz::SetupTrafficViz(loader.road_map, 10);

    /****************************************************************************/

    // ------------------- vehicle 1 ---------------------- //

    CovarMatrix2d pos_covar1;
    pos_covar1 << 2, 0,
        0, 2;
    VehicleEstimation veh1({35, 59, -7 / 180.0 * M_PI}, pos_covar1, 10, 3 * 3);

    // ------------------- vehicle 2 ---------------------- //

    CovarMatrix2d pos_covar2;
    pos_covar2 << 1, 0,
        0, 1;
    VehicleEstimation veh2({89, 52, -7 / 180.0 * M_PI}, pos_covar2, 10, 2 * 2);

    // ------------------- vehicle 3 ---------------------- //

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, pos_covar3, 10, 1 * 1);

    // ------------------- vehicle 4 ---------------------- //

    CovarMatrix2d pos_covar4;
    pos_covar4 << 1, 0,
        0, 1;
    VehicleEstimation veh4({52, 35, -95 / 180.0 * M_PI}, pos_covar4, 10, 2 * 2);

    // ------------------- vehicle 5 ---------------------- //

    CovarMatrix2d pos_covar5;
    pos_covar5 << 1, 0,
        0, 1;
    VehicleEstimation veh5({40, 64, 171 / 180.0 * M_PI}, pos_covar5, 10, 2 * 2);

    /****************************************************************************/

    // trajectory planning for ego vehicle
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(2, 0.74, 3);

    VehicleEstimation ego_veh({56.5, 36, 85.0 / 180.0 * M_PI}, 15);

    stopwatch::StopWatch timer;
    std::vector<StateLattice> path;
    // auto graph = LatticeGraph::Search(path, ego_chn, {2, 0}, 8);
    auto graph = LatticeGraph::Search(path, ego_chn, ego_veh.GetPose(), 8);
    std::cout << "search finished in " << timer.toc() << " seconds" << std::endl;

    if (path.empty())
    {
        std::cout << "failed to find a path" << std::endl;
        return -1;
    }

    ReferenceTrajectory traj(path);
    traj.GenerateConstantSpeedProfile(ego_veh.GetSpeed());

    LookaheadZone zone(traj);

    TrafficViz::ShowLatticePathWithLookaheadZone(path, zone, *ego_chn.get(), "path with lookahead");

    /****************************************************************************/

    timer.tic();
    ThreatEvaluation ranker(loader.road_map, loader.traffic_map);
    ranker.SetTrafficConfiguration({veh1, veh2, veh3, veh4, veh5});
    ranker.SetEgoConfiguration(ego_veh, ego_chn, zone);
    ranker.Evaluate(6);
    std::cout << "evaluation finished in " << timer.toc() << " seconds" << std::endl;

    // TrafficViz::ShowTrafficChannelWithThreatField(*ego_chn.get(), ranker.field_, 4, true, "lattice_in_threat_field", true);

    return 0;
}