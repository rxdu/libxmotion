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

    // // ------------------- vehicle 1 ---------------------- //

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

    // std::vector<Polygon> vehs = {veh1.GetFootprint(), veh2.GetFootprint(), veh3.GetFootprint(), veh4.GetFootprint(), veh5.GetFootprint()};
    // TrafficViz::ShowVehicle(vehs);

    // for(auto& veh:vehs)
    //     TrafficViz::ShowVehicleInChannel(veh, *ego_chn1.get());

    // for(auto chn : loader.traffic_map->GetAllTrafficChannels())
    //     TrafficViz::ShowVehicleInChannel(veh1.GetFootprint(), *chn.get());

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
    std::cout << "number of vertices: " << graph->GetTotalVertexNumber() << std::endl;

    // LightViz::ShowStateLattice(lattices);
    // TrafficViz::ShowLatticeInTrafficChannel(lattices, *ego_chn.get(), "lattice graph", true);

    /****************************************************************************/

    timer.tic();

    ThreatField field(loader.road_map, loader.traffic_map);
    field.AddVehicleEstimations({veh1, veh2, veh3, veh4, veh5});

    Pose2d ego_pose(57, 36, 85.0 / 180.0 * M_PI);
    field.SetupThreatField(ego_pose, ego_chn);

    //////////////////////////////////////////////////

    field.ComputeThreatField(4);

    std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    std::cout << "------------- all calculation finished -------------" << std::endl;

    TrafficViz::ShowLatticeInThreatField(lattices, ego_chn.get(), field, 4, true, "lattice_in_threat_field", true);

    //////////////////////////////////////////////////

    // TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(2), true);

    // for (int i = 0; i < 9; i++)
    // {
    //     field.ComputeThreatField(i);
    //     TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(i), true);
    // }

    TrafficViz::ShowTrafficChannelWithThreatField(ego_chn.get(), field, 4, true, "lattice_in_threat_field", true);

    return 0;
}