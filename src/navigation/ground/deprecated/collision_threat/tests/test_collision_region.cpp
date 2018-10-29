#include <iostream>

#include "road_map/road_map.hpp"

#include "traffic_map/traffic_map.hpp"
#include "threat_field/collision_field.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    /********** create road map **********/
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    RoadMapViz::SetupRoadMapViz(map);

    /********** create traffic map **********/

    timer.tic();

    std::shared_ptr<TrafficMap> traffic_map = std::make_shared<TrafficMap>(map);
    traffic_map->DiscretizeTrafficRegions(1.2 * 4);

    std::cout << "traffic flow map constructed in " << timer.toc() << " seconds" << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////

    auto ego_chn = traffic_map->GetTrafficChannel("s4", "s1");
    auto ego_flow = TrafficFlow(ego_chn);

    // auto cflows = traffic_map->FindConflictingFlows("s4", "s1");
    // std::cout << "number of conflicting cflows: " << cflows.size() << std::endl;
    // for (auto &tf : cflows)
    // {
    //     auto blks = tf.GetAllLaneBlocks();
    //     std::cout << " - number of blocks: " << blks.size() << std::endl;
    //     RoadMapViz::ShowVehicleFootprints(blks, 10);
    // }

    std::vector<TrafficFlow *> cflows;
    auto other_chn = traffic_map->GetTrafficChannel("s6", "s3");
    auto other_flow = TrafficFlow(other_chn);
    cflows.push_back(&other_flow);

    timer.tic();
    // auto labeled = ego_flow.CheckConflicts(cflows);
    // auto labeled = traffic_map->CheckCollision(&ego_flow, &other_flow);
    auto poses = traffic_map->BackTrackCollision(&ego_flow, cflows, 10);
    std::cout << "conflict analysis finished in " << timer.toc() << " seconds" << std::endl;

    // RoadMapViz::ShowLabledTrafficFlows(&ego_flow, cflows, 10, "collision", true);

    /////////////////////////////////////////////////////

    VehiclePose bkpose = poses.front().pose;
    std::cout << "collision region: " << bkpose.x << " , " << bkpose.y << std::endl;
    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(map->xmin_, map->xmax_, map->ymin_, map->ymax_);
    GaussianPositionVelocityThreat threat_model(bkpose.x, bkpose.y, 10, -1, 3, 0.5);
    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(bkpose.x, bkpose.y, 10, 0);
    participant->threat_func = threat_model;
    cfield->AddTrafficParticipant(0, participant);

    RoadMapViz::ShowCollisionFieldWithTrafficFlows(cfield, &ego_flow, cflows);

    return 0;
}