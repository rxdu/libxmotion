#include <iostream>

#include "road_map/road_map.hpp"
#include "traffic_map/threat_ranking.hpp"

#include "threat_field/collision_field.hpp"

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
    ranker.SetCaseLabel("theat-case1");

    /********** create observations **********/
    std::vector<MMStateEst> ests;
    
    MMStateEst mpt0(85, 58, 8.5, -0, 1, 1);
    mpt0.id = 0;
    ests.push_back(mpt0);

    MMStateEst mpt1(38, 59, 7.5, 0, 1, 1); 
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

    map->GetAllLaneBoundPolylines();
    map->GetAllLaneCenterPolylines();

    /********** start analysis **********/
    ranker.Analyze();

    // LightViz::ShowLanePolylines(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines());

    return 0;
}