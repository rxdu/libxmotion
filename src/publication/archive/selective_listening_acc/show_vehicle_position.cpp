#include <iostream>

#include "road_map/road_map.hpp"
#include "collision_threat/motion_model.hpp"

#include "threat_field/collision_field.hpp"

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

    // ego vehicle
    MMStateEst mpt0(57, 36, 5, 1, 1, 1);
    model.AddVehicleStateEstimate(mpt0);

    MMStateEst mpt1(85, 58, 5, 1, 1, 1);
    model.AddVehicleStateEstimate(mpt1);

    MMStateEst mpt2(38, 59, 1, 0.25, 1, 1);
    model.AddVehicleStateEstimate(mpt2);

    model.MergePointsToNetwork();

    auto cfields = model.GenerateCollisionField();
    RoadMapViz::ShowCollisionField(cfields);

    return 0;
}