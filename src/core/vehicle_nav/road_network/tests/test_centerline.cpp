#include <iostream>

#include "road_network/road_map.hpp"

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm", 10);

    if (!map.MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    // LightViz::ShowMatrixAsColorMap(map.GetFullLaneBoundaryGrid()->GetGridMatrix(true), "roadnetwork", true);
    // LightViz::ShowMatrixAsColorMap(map.GetFullCenterLineGrid()->GetGridMatrix(true) + map.GetFullDrivableAreaGrid()->GetGridMatrix(true), "centerline", true);

    // create a threat field
    CollisionField cfield(map.grid_size_x_, map.grid_size_y_);
    cfield.SetOriginCoordinate(0, 0);

    // add traffic participant
    auto pt0 = std::make_shared<CollisionField::TrafficParticipantType>(map.grid_size_x_, map.grid_size_y_);
    pt0->SetPositionVelocity(220, 105, 1, 0.3);
    cfield.AddTrafficParticipant(0, pt0);

    auto pt1 = std::make_shared<CollisionField::TrafficParticipantType>(map.grid_size_x_, map.grid_size_y_);
    pt1->SetPositionVelocity(720, 120, -1, -0.25);
    cfield.AddTrafficParticipant(1, pt1);

    auto pt2 = std::make_shared<CollisionField::TrafficParticipantType>(map.grid_size_x_, map.grid_size_y_);
    pt2->SetPositionVelocity(550, 95, -1, -0.25);
    cfield.AddTrafficParticipant(2, pt2);

    auto pt3 = std::make_shared<CollisionField::TrafficParticipantType>(map.grid_size_x_, map.grid_size_y_);
    pt3->SetPositionVelocity(580, 125, -1, 0.4);
    cfield.AddTrafficParticipant(3, pt3);

    // update collision field before use
    cfield.UpdateCollisionField();

    std::cout << "collision field created" << std::endl;

    ScalarFieldMatrix mat = cfield.GenerateFieldMatrix(0, 1, 0, 1, true);
    // LightViz::ShowMatrixAsColorMap(mat.z);

    LightViz::ShowMatrixAsColorMap(map.GetFullCenterLineGrid()->GetGridMatrix(true) 
        + map.GetFullDrivableAreaGrid()->GetGridMatrix(true) + 
        cfield.GetGridMatrix(true), "centerline", true);

    return 0;
}