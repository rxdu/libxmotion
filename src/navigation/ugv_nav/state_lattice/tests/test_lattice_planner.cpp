#include <memory>
#include <iostream>
#include <cstdint>

#include "road_network/road_map.hpp"

#include "state_lattice/lattice_manager.hpp"
#include "state_lattice/lattice_planner.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

using namespace librav;

int main()
{
    /********** create road map **********/
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm", 10);

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    std::vector<std::string> sinks = {"s1", "s3", "s5"};
    std::vector<std::string> sources = {"s2", "s4", "s6"};
    map->SetTrafficSinkSource(sinks, sources);

    auto ego_path = map->FindShortestRouteName("s4", "s1");
    auto drivable_mask = map->GetLaneDrivableGrid(ego_path);

    std::shared_ptr<LatticeManager> lm = std::make_shared<LatticeManager>();

    lm->LoadPrimitivesFromFile("/home/rdu/mp.20180809031850.data");

    // auto new_base = lm->primitives_[35];
    // std::vector<MotionPrimitive> new_mps = lm->TransformAllPrimitives(lm->primitives_, new_base.GetFinalNode().x, new_base.GetFinalNode().y, new_base.GetFinalNode().theta);
    // lm->SavePrimitivesToFile(new_base.GetFinalNode(), new_mps, "mp_trans");

    LatticePlanner planner(lm, map);
    planner.SetDrivableAreaMask(drivable_mask);

    // LatticeNode start(0, 0, 0);
    // LatticeNode goal(20, 20, -M_PI / 6.0);
    // LatticeNode start(57, 36, 80.0 / 180.0 * M_PI);
    // LatticeNode goal(20, 66.5, M_PI);
    LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    // LatticeNode goal(23, 65.8, 180.0 / 180.0 * M_PI);
    LatticeNode goal(22, 66.8, 180.0 / 180.0 * M_PI);

    // auto path = planner.Search(start, goal);
    auto path = planner.AStarSearch(start, goal);

    // lm->SavePrimitivesToFile(path, "path");

    if (!path.empty())
    {
        std::vector<RectGridIndex> waypoints;
        for (auto &mp : path)
        {
            for (auto &nd : mp.nodes)
            {
                auto grid_pos = map->coordinate_.ConvertToGridPixel(CartCooridnate(nd.x, nd.y));

                waypoints.push_back(RectGridIndex(grid_pos.x, grid_pos.y));
            }
        }
        LightViz::ShowPathOnMatrixAsColorMap(drivable_mask->GetGridMatrix(true), waypoints, "lplanner", true);
    }

    return 0;
}