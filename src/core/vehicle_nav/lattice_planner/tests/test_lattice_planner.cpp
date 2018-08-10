#include <memory>
#include <iostream>
#include <cstdint>

#include "road_map/road_map.hpp"

#include "lattice_planner/lattice_manager.hpp"
#include "lattice_planner/lattice_planner.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

using namespace librav;

int main()
{
    /********** create road map **********/
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

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
    fp.AddPoint(0.5, 1.0);
    fp.AddPoint(-0.5, 1.0);
    fp.AddPoint(-0.5, -1.0);
    fp.AddPoint(0.5, -1.0);
    planner.SetVehicleFootprint(fp);

    // LatticeNode start(0, 0, 0);
    // LatticeNode goal(20, 20, -M_PI / 6.0);
    // LatticeNode start(57, 36, 80.0 / 180.0 * M_PI);
    // LatticeNode goal(20, 66.5, M_PI);
    LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    // LatticeNode goal(23, 65.8, 180.0 / 180.0 * M_PI);
    LatticeNode goal(22, 66.8, 180.0 / 180.0 * M_PI);

    // auto path = planner.Search(start, 5);
    auto path = planner.AStarSearch(start, goal);

    // lm->SavePrimitivesToFile(path, "path");

    // if (!path.empty())
    // {
    //     std::vector<GridCoordinate> waypoints;
    //     for (auto &mp : path)
    //     {
    //         for (auto &nd : mp.nodes)
    //         {
    //             auto grid_pos = map->coordinate_.ConvertToGridPixel(CartCooridnate(nd.x, nd.y));

    //             waypoints.push_back(GridCoordinate(grid_pos.x, grid_pos.y));
    //         }
    //     }
    //     LightViz::ShowPathOnMatrixAsColorMap(drivable_mask->GetGridMatrix(true), waypoints, "lplanner", true);
    // }

    return 0;
}