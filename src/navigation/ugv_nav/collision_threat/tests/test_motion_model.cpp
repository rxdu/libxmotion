#include <iostream>

#include "road_map/road_map.hpp"

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"
#include "collision_threat/motion_model.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    RoadMapViz::SetupRoadMapViz(map);

    // std::vector<std::string> sinks = {"s1", "s3", "s5"};
    // std::vector<std::string> sources = {"s2", "s4", "s6"};
    // map->SetTrafficSinkSource(sinks, sources);

    MotionModel model(map);

    // MMStateEst mpt0(4, 68, -1, -0.25, 1, 1);

    // good test case 1
    MMStateEst mpt0(85, 53, 5, 1, 1, 1);
    model.AddVehicleStateEstimate(mpt0);

    MMStateEst mpt1(57, 36, 5, 1, 1, 1);
    model.AddVehicleStateEstimate(mpt1);

    // // good test case 2
    // MMStateEst mpt1(47, 58, 1, 0.25, 15, 15);
    // model.AddVehicleStateEstimate(mpt1);

    // debugging
    // MMStateEst mpt0(55, 56, 1, 0.25, 5, 5);
    // model.AddVehicleStateEstimate(mpt0);

    // good starting position (57, 36)
    // good finishing position (20, 66.5)
    // MMStateEst mpt0(10, 68.5, 1, 0.25, 5, 5);
    // MMStateEst mpt0(58, 49, 1, 0.25, 5, 5);

    // model.AddVehicleStateEstimate(mpt0);

    model.MergePointsToNetwork();

    auto cfields = model.GenerateCollisionField();
    // LightViz::ShowCollisionField(cfields);
    // LightViz::ShowCollisionFieldInRoadMap(cfields, map);
    RoadMapViz::ShowCollisionField(cfields);

    // auto cfield = model.GeneratePredictedCollisionField(1.0);
    // LightViz::ShowCollisionFieldInRoadMap(cfield, map);

    // auto cfield2 = model.GeneratePredictedCollisionField(0.5);
    // LightViz::ShowCollisionFieldInRoadMap(cfield2, map);

    // for (int i = 0; i < 10; ++i)
    // {
    //     auto cfield = model.GeneratePredictedCollisionField(2 * (i + 1));
    //     // LightViz::ShowCollisionField(cfield);
    //     LightViz::ShowCollisionFieldInRoadMap(cfield, map);
    // }

    // pt0->SetParameters(700, 118, -1, -0.25);
    // pt1->SetParameters(720, 120, -1, -0.25);
    // pt2->SetParameters(550, 95, -1, -0.25);
    // pt3->SetParameters(580, 125, -1, 0.4);

    return 0;
}