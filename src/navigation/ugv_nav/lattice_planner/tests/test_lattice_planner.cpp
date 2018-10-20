#include <memory>
#include <iostream>
#include <cstdint>

#include "road_map/road_map.hpp"

#include "lattice_planner/lattice_planner.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    /********** create road map **********/
    // std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    RoadMapViz::SetupRoadMapViz(map);

    std::shared_ptr<LatticeManager> lm = std::make_shared<LatticeManager>();
    lm->LoadPrimitivesFromFile("/home/rdu/mp.1s20mph-3.data");

    LatticePlanner planner(lm, map);

    auto ego_route = map->FindShortestRouteName("s4", "s1");
    planner.SetEgoPlanningRoute(ego_route);

    Polygon fp;
    // fp.AddPoint(0.6, 1.0);
    // fp.AddPoint(-0.6, 1.0);
    // fp.AddPoint(-0.6, -1.0);
    // fp.AddPoint(0.6, -1.0);
    // this set works
    // fp.AddPoint(0.5, 1.0);
    // fp.AddPoint(-0.5, 1.0);
    // fp.AddPoint(-0.5, -1.0);
    // fp.AddPoint(0.5, -1.0);
    // this set works
    // fp.AddPoint(0.55, 1.2);
    // fp.AddPoint(-0.55, 1.2);
    // fp.AddPoint(-0.55, -1.2);
    // fp.AddPoint(0.55, -1.2);
    // fp.AddPoint(1.2 * 2, 0.9);
    // fp.AddPoint(1.2 * 2, -0.9);
    // fp.AddPoint(-1.2 * 2, -0.9);
    // fp.AddPoint(-1.2 * 2, 0.9);
    fp.AddPoint(0.55, 1.2);
    fp.AddPoint(-0.55, 1.2);
    fp.AddPoint(-0.55, -1.2);
    fp.AddPoint(0.55, -1.2);
    // fp.AddPoint(1.2 * 0.8, -0.55);
    // fp.AddPoint(1.2 * 0.8, 0.55);
    // fp.AddPoint(-1.2 * 0.8, 0.55);
    // fp.AddPoint(-1.2 * 0.8, -0.55);
    planner.SetVehicleFootprint(fp);

    // LatticeNode start(0, 0, 0);
    // LatticeNode goal(20, 20, -M_PI / 6.0);
    // LatticeNode start(57, 36, 80.0 / 180.0 * M_PI);
    // LatticeNode goal(20, 66.5, M_PI);
    // LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    // LatticeNode goal(23, 65.8, 180.0 / 180.0 * M_PI);
    // LatticeNode goal(22, 66.8, 180.0 / 180.0 * M_PI);
    // LatticeNode goal(4, 68, 180.0 / 170.0 * M_PI);

    LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    LatticeNode goal(4, 68, 170.0 / 180.0 * M_PI);

    auto fp_start = fp.TransformRT(57, 36, 85.0 / 180.0 * M_PI);
    auto fp_final = fp.TransformRT(4, 68, 170.0 / 180.0 * M_PI);

    // auto path = planner.Search(start, 5);
    auto path = planner.AStarSearch(start, goal, 8, 10);
    // auto path = planner.BFSSearch(start, goal, 8, 10);

    ///////////////////////// Visualization /////////////////////////

    std::vector<Polygon> polys;
    polys.push_back(fp_start);
    polys.push_back(fp_final);

    auto path_line = planner.ConvertPathToPolyline(path);

    // RoadMapViz::ShowVehicle(new_fp);
    // RoadMapViz::ShowVehicleFootprints(polys);
    RoadMapViz::ShowVehiclePath(path_line, polys);

    return 0;
}