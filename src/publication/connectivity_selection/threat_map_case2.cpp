#include <iostream>

#include "road_map/road_map.hpp"

#include "traffic_map/traffic_map.hpp"
#include "threat_field/collision_field.hpp"
#include "collision_threat/collision_threat.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    /********** create road map **********/
    std::shared_ptr<RoadMap> road_map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!road_map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    RoadMapViz::SetupRoadMapViz(road_map);

    /********** create traffic map **********/

    timer.tic();

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(road_map);
    traffic_map->DiscretizeTrafficRegions(1.2 * 4);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////

    CollisionThreat threat(road_map, traffic_map);

    int32_t ego_id = 8;
    double average_spd = 10;

    // auto ego_chn = traffic_map->GetTrafficChannel("s4", "s3");
    auto ego_chn = traffic_map->GetTrafficChannel("s4", "s1");
    auto ego_flow = TrafficFlow(ego_chn);
    ego_flow.AssignConstantSpeedProfile(ego_id, 0.8);
    threat.SetEgoTrafficFlow(&ego_flow);

    auto cfield = threat.EvaluateThreat(average_spd);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    // threat.CalculateThreatLevel(cfield, 79, 58, 1, 1);
    // threat.CalculateThreatLevel(cfield, 41, 58, 1, 1);

    Polygon ego_polygon = ego_chn.lane_blocks[ego_id];

    auto conflicting_flows = threat.GetConflictingFlows();
    RoadMapViz::ShowCollisionFieldWithTrafficFlows(ego_polygon, cfield, &ego_flow, conflicting_flows, 10, "threat_field_slow", true);

    return 0;
}