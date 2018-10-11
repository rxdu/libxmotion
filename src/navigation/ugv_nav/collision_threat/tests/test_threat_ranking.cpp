#include <iostream>

#include "road_map/road_map.hpp"

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "collision_threat/threat_ranking.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

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

    // std::vector<std::string> sinks = {"s1", "s3", "s5"};
    // std::vector<std::string> sources = {"s2", "s4", "s6"};
    // map->SetTrafficSinkSource(sinks, sources);

    /********** create threat ranker **********/

    ThreatRanking ranker(map);

    /********** create observations **********/
    std::vector<MMStateEst> ests;
    // good test case 1
    MMStateEst mpt0(70, 60, -10, -2.5, 1, 1);//case 1
    // MMStateEst mpt0(70, 60, -10 / 2.5, -2.5 / 2.5, 1, 1); // case 2
    mpt0.id = 0;
    ests.push_back(mpt0);

    // good test case 2
    MMStateEst mpt1(47, 58, 10, 2.5, 1, 1); // case 1
    // MMStateEst mpt1(47, 58, 10 * 1.3, 2.5 * 1.3, 1, 1); // case 2
    mpt1.id = 1;
    ests.push_back(mpt1);

    ranker.AddStateEstimations(ests);

    /********** set ego vehicle info **********/
    Polygon fp;
    fp.AddPoint(0.55, 1.2);
    fp.AddPoint(-0.55, 1.2);
    fp.AddPoint(-0.55, -1.2);
    fp.AddPoint(0.55, -1.2);
    ranker.SetEgoVehicleFootprint(fp);

    ranker.SetEgoDesiredPath("s4", "s1");
    ranker.SetEgoStartState(57, 36, 85.0 / 180.0 * M_PI);
    ranker.SetEgoGoalState(4, 68, 170.0 / 180.0 * M_PI);

    // ranker.SetEgoStartState(57, 36, 80.0 / 180.0 * M_PI);
    // ranker.SetEgoGoalState(20, 66.5, 175.0 / 180.0 * M_PI);
    // LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    // LatticeNode goal(4, 68, 180.0 / 170.0 * M_PI);

    // pt0->SetParameters(700, 118, -1, -0.25);
    // pt1->SetParameters(720, 120, -1, -0.25);
    // pt2->SetParameters(550, 95, -1, -0.25);
    // pt3->SetParameters(580, 125, -1, 0.4);

    map->GetAllLaneBoundPolylines();
    map->GetAllLaneCenterPolylines();

    /********** start analysis **********/
    ranker.Analyze();

    // LightViz::ShowLanePolylines(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines());

    return 0;
}