#include <iostream>

#include "road_network/road_map.hpp"

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"
#include "threat_ranking/motion_model.hpp"
#include "threat_ranking/threat_ranking.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

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

    /********** create threat ranker **********/

    ThreatRanking ranker(map);

    /********** create observations **********/
    std::vector<MMStateEst> ests;
    // good test case 1
    MMStateEst mpt0(70, 60, -1, -0.25, 15, 15);
    ests.push_back(mpt0);

    // good test case 2
    MMStateEst mpt1(47, 58, 1, 0.25, 15, 15);
    ests.push_back(mpt1);

    ranker.AddStateEstimations(ests);
    ranker.SetEgoDesiredPath("s4", "s1");
    // ranker.SetEgoStartState(57, 36, 80.0 / 180.0 * M_PI);
    // ranker.SetEgoGoalState(20, 66.5, 175.0 / 180.0 * M_PI);

    ranker.SetEgoStartState(57, 36, 85.0 / 180.0 * M_PI);
    ranker.SetEgoGoalState(20, 66.5, 180.0 / 180.0 * M_PI);

    // ranker.SetEgoStartState(58, 49, 85.0 / 180.0 * M_PI);
    // ranker.SetEgoGoalState(10, 68, 175.0 / 180.0 * M_PI);

    ranker.Analyze();

    if (!ranker.path_.empty())
    {
        std::vector<GridCoordinate> waypoints;
        for (auto &mp : ranker.path_)
        {
            for (auto &nd : mp.nodes)
            {
                auto grid_pos = map->coordinate_.ConvertToGridPixel(CartCooridnate(nd.x, nd.y));

                waypoints.push_back(GridCoordinate(grid_pos.x, grid_pos.y));
            }
        }
        LightViz::ShowPathOnMatrixAsColorMap(ranker.drivable_mask_->GetGridMatrix(true), waypoints, "ego_drivable", true);
    }

    // model.MergePointsToNetwork();

    // model.GenerateCollisionField();
    // LightViz::ShowMatrixAsColorMap(model.GetThreatFieldVisMatrix(), "tfield", true);

    // for (int i = 0; i < 10; ++i)
    // {
    //     model.GeneratePredictedCollisionField(2 * (i + 1));
    //     LightViz::ShowMatrixAsColorMap(model.GetThreatFieldVisMatrix(), "tfield" + std::to_string(i), true);
    // }

    // LightViz::ShowMatrixAsColorMap(map->GetFullLaneBoundaryGrid()->GetGridMatrix(true), "roadnetwork", true);
    // LightViz::ShowMatrixAsColorMap(map->GetFullCenterLineGrid()->GetGridMatrix(true) + map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "centerline", true);

    // pt0->SetParameters(700, 118, -1, -0.25);
    // pt1->SetParameters(720, 120, -1, -0.25);
    // pt2->SetParameters(550, 95, -1, -0.25);
    // pt3->SetParameters(580, 125, -1, 0.4);

    // LightViz::ShowMatrixAsColorMap(ranker.drivable_mask_->GetGridMatrix(true), "ego_drivable", true);

    return 0;
}