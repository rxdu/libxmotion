#include <iostream>

#include "road_network/road_map.hpp"

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"
#include "threat_ranking/motion_model.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm", 10);

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    std::vector<std::string> sinks = {"s1", "s3", "s5"};
    std::vector<std::string> sources = {"s2", "s4", "s6"};

    map->SetTrafficSinkSource(sinks, sources);

    MotionModel model(map);

    MMStateEst mpt0(70, 60, -1, -0.25, 15, 15);
    model.AddVehicleStateEstimate(mpt0);

    model.MergePointsToNetwork();
    model.GenerateCollisionField();

    // LightViz::ShowMatrixAsColorMap(map->GetFullLaneBoundaryGrid()->GetGridMatrix(true), "roadnetwork", true);
    // LightViz::ShowMatrixAsColorMap(map->GetFullCenterLineGrid()->GetGridMatrix(true) + map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "centerline", true);

    // pt0->SetPositionVelocity(700, 118, -1, -0.25);
    // pt1->SetPositionVelocity(720, 120, -1, -0.25);
    // pt2->SetPositionVelocity(550, 95, -1, -0.25);
    // pt3->SetPositionVelocity(580, 125, -1, 0.4);

    LightViz::ShowMatrixAsColorMap(model.GetThreatFieldVisMatrix(), "tfield", true);

    return 0;
}