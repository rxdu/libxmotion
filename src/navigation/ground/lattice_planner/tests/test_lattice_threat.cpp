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
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane_horizontal.osm");
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/single_bidirectional_lane.osm");
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/short_segment.osm");
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    TrafficViz::SetupTrafficViz(loader.road_map, 10);

    // TrafficViz::ShowLanes(true, 5, "test_lane", true);
    // for (auto &chn : map->traffic_map_->GetAllTrafficChannels())
    // {
    //     // TrafficViz::ShowTrafficChannelCenterline(chn);
    //     chn->PrintInfo();
    //     TrafficViz::ShowTrafficChannel(*chn.get(), 5);
    // }

    /****************************************************************************/

    // discretize lane
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    // all_channels[1]->DiscretizeChannel(5, 1.2, 5);
    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(10, 1.2, 1);

    // TrafficViz::ShowTrafficChannel(*all_channels[1].get());
    // TrafficViz::ShowTrafficChannel(*all_channels[1].get(), 20, "horizontal_lane", true);

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

    // // ------------------- vehicle 1 ---------------------- //

    CovarMatrix2d pos_covar1;
    pos_covar1 << 2, 0,
        0, 2;
    auto ego_chn1 = loader.traffic_map->GetAllTrafficChannels()[4];

    VehicleEstimation veh1({35, 59, -7 / 180.0 * M_PI}, 10, ego_chn1);
    veh1.SetPositionVariance(pos_covar1);
    veh1.SetSpeedVariance(3 * 3);

    // ------------------- vehicle 2 ---------------------- //

    CovarMatrix2d pos_covar2;
    pos_covar2 << 1, 0,
        0, 1;
    auto ego_chn2 = loader.traffic_map->GetAllTrafficChannels()[4];

    VehicleEstimation veh2({89, 52, -7 / 180.0 * M_PI}, 10, ego_chn2);
    veh2.SetPositionVariance(pos_covar2);
    veh2.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 3 ---------------------- //

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    auto ego_chn3 = loader.traffic_map->GetAllTrafficChannels()[0];

    VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, 10, ego_chn3);
    veh3.SetPositionVariance(pos_covar3);
    veh3.SetSpeedVariance(1 * 1);

    // ------------------- vehicle 4 ---------------------- //

    CovarMatrix2d pos_covar4;
    pos_covar4 << 1, 0,
        0, 1;
    auto ego_chn4 = loader.traffic_map->GetAllTrafficChannels()[5];

    VehicleEstimation veh4({52, 35, -95 / 180.0 * M_PI}, 10, ego_chn4);
    veh4.SetPositionVariance(pos_covar4);
    veh4.SetSpeedVariance(2 * 2);

    // ------------------- vehicle 5 ---------------------- //

    CovarMatrix2d pos_covar5;
    pos_covar5 << 1, 0,
        0, 1;
    auto ego_chn5 = loader.traffic_map->GetAllTrafficChannels()[0];

    VehicleEstimation veh5({40, 64, 171 / 180.0 * M_PI}, 10, ego_chn5);
    veh5.SetPositionVariance(pos_covar5);
    veh5.SetSpeedVariance(2 * 2);

    // std::vector<Polygon> vehs = {veh1.GetFootprint(), veh2.GetFootprint(), veh3.GetFootprint(), veh4.GetFootprint(), veh5.GetFootprint()};
    // TrafficViz::ShowVehicle(vehs);

    // for(auto& veh:vehs)
    //     TrafficViz::ShowVehicleInChannel(veh, *ego_chn1.get());

    // for(auto chn : loader.traffic_map->GetAllTrafficChannels())
    //     TrafficViz::ShowVehicleInChannel(veh1.GetFootprint(), *chn.get());

    /****************************************************************************/

    timer.tic();

    ThreatField field(loader.road_map, loader.traffic_map);
    field.AddVehicleEstimations({veh1, veh2, veh3, veh4, veh5});

    field.SetupThreatField(ego_chn);

    //////////////////////////////////////////////////

    field.ComputeThreatField(4);

    std::cout << "occupancy estimation calculated in " << timer.toc() << std::endl;

    std::cout << "------------- all calculation finished -------------" << std::endl;

    TrafficViz::ShowLatticeInThreatField(lattices, *ego_chn.get(), field, 4, true, "lattice_in_threat_field", true);

    //////////////////////////////////////////////////

    // TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(2), true);

    // for (int i = 0; i < 9; i++)
    // {
    //     field.UpdateThreatField(i);
    //     TrafficViz::ShowThreatField(field, true, "occupancy_estimation" + std::to_string(i), true);
    // }

    return 0;
}