#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "connectivity/threat_evaluation.hpp"

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

    // // ------------------- vehicle 1 ---------------------- //

    CovarMatrix2d pos_covar1;
    pos_covar1 << 2, 0,
        0, 2;
    auto v1_chn = loader.traffic_map->GetAllTrafficChannels()[4];

    VehicleEstimation veh1({35, 59, -7 / 180.0 * M_PI}, 10, v1_chn);
    veh1.SetPositionVariance(pos_covar1);
    veh1.SetSpeedVariance(3 * 3);

    // ------------------- vehicle 2 ---------------------- //

    CovarMatrix2d pos_covar2;
    pos_covar2 << 1, 0,
        0, 1;
    auto v2_chn = loader.traffic_map->GetAllTrafficChannels()[4];

    VehicleEstimation veh2({89, 52, -7 / 180.0 * M_PI}, 10, v2_chn);
    veh2.SetPositionVariance(pos_covar2);
    veh2.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 3 ---------------------- //

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    auto v3_chn = loader.traffic_map->GetAllTrafficChannels()[0];

    VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, 10, v3_chn);
    veh3.SetPositionVariance(pos_covar3);
    veh3.SetSpeedVariance(1 * 1);

    // ------------------- vehicle 4 ---------------------- //

    CovarMatrix2d pos_covar4;
    pos_covar4 << 1, 0,
        0, 1;
    auto v4_chn = loader.traffic_map->GetAllTrafficChannels()[5];

    VehicleEstimation veh4({52, 35, -95 / 180.0 * M_PI}, 10, v4_chn);
    veh4.SetPositionVariance(pos_covar4);
    veh4.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 5 ---------------------- //

    CovarMatrix2d pos_covar5;
    pos_covar5 << 1, 0,
        0, 1;
    auto v5_chn = loader.traffic_map->GetAllTrafficChannels()[0];

    VehicleEstimation veh5({40, 64, 171 / 180.0 * M_PI}, 10, v5_chn);
    veh5.SetPositionVariance(pos_covar5);
    veh5.SetSpeedVariance(2 * 2);

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(10, 0.74, 5);

    stopwatch::StopWatch timer;

    auto graph = LatticeGraph::Construct(ego_chn, {0, 0}, 11);
    std::cout << "graph constructed in " << timer.toc() << " seconds" << std::endl;

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    std::cout << "number of vertices: " << graph->GetGraphVertexNumber() << std::endl;

    // LightViz::ShowStateLattice(lattices);
    // TrafficViz::ShowLatticeInTrafficChannel(lattices, *ego_chn.get(), "lattice graph", true);

    /****************************************************************************/

    // timer.tic();

    // ThreatField field(loader.road_map, loader.traffic_map);
    // field.AddVehicleEstimations({veh1, veh2, veh3, veh4, veh5});

    // field.SetupThreatField(ego_chn);

    // //////////////////////////////////////////////////

    // field.ComputeThreatField(4);

    // std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    // std::cout << "------------- all calculation finished -------------" << std::endl;

    // TrafficViz::ShowLatticeInThreatField(lattices, *ego_chn.get(), field, 4, true, "lattice_in_threat_field", true);

    //////////////////////////////////////////////////

    // TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(2), true);

    // for (int i = 0; i < 9; i++)
    // {
    //     field.ComputeThreatField(i);
    //     TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(i), true);
    // }

    // TrafficViz::ShowTrafficChannelWithThreatField(*ego_chn.get(), field, 4, true, "lattice_in_threat_field", true);

    ThreatEvaluation ranker(loader.road_map, loader.traffic_map);
    ranker.SetTrafficConfiguration({veh1, veh2, veh3, veh4, veh5}, ego_chn);
    ranker.Evaluate(4);

    TrafficViz::ShowTrafficChannelWithThreatField(*ego_chn.get(), ranker.field_, 4, true, "lattice_in_threat_field", true);

    return 0;
}