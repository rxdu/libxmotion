#include <iostream>
#include <memory>

#include "field/collision_field.hpp"
#include "field/traffic_participant.hpp"
#include "field/threat_distribution.hpp"
#include "traffic_flowfield_search.hpp"

#include "fastplot/fastplot.hpp"
#include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    const int32_t fsize_x = 500;
    const int32_t fsize_y = 222;

    const int32_t half_lane_width = 18;
    // lane1-4 from bottom -> top
    const int32_t lane1_y = 74 + half_lane_width;
    const int32_t lane2_y = 111 + half_lane_width;
    const int32_t lane3_y = 148 + half_lane_width;
    const int32_t lane4_y = 185 + half_lane_width;

    stopwatch::StopWatch timer;

    timer.tic();

    // create an empty collision field
    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(fsize_x, fsize_y);
    cfield->SetOriginCoordinate(0, 0);

    const int8_t vnum = 5;
    const int32_t vdist = 80;

    // add traffic participant
    // cars on lane 1
    std::vector<std::shared_ptr<TrafficParticipantType>> vehicle_group1;
    for (int i = 0; i < vnum; ++i)
    {
        auto tp = std::make_shared<TrafficParticipantType>(fsize_x, fsize_y);
        tp->SetParameters(vdist * (i + 1), lane1_y, 15, 0);

        vehicle_group1.push_back(tp);
    }

    // cars on lane 2
    std::vector<std::shared_ptr<TrafficParticipantType>> vehicle_group2;
    for (int i = 0; i < vnum; ++i)
    {
        auto tp = std::make_shared<TrafficParticipantType>(fsize_x, fsize_y);
        tp->SetParameters(vdist * (i + 1), lane2_y, 15, 0);

        vehicle_group2.push_back(tp);
    }

    // cars on lane 3
    std::vector<std::shared_ptr<TrafficParticipantType>> vehicle_group3;
    for (int i = 0; i < vnum; ++i)
    {
        auto tp = std::make_shared<TrafficParticipantType>(fsize_x, fsize_y);
        tp->SetParameters(vdist * (i + 1), lane3_y, 15, 0);

        vehicle_group3.push_back(tp);
    }

    // cars on lane 4
    std::vector<std::shared_ptr<TrafficParticipantType>> vehicle_group4;
    for (int i = 0; i < vnum; ++i)
    {
        auto tp = std::make_shared<TrafficParticipantType>(fsize_x, fsize_y);
        tp->SetParameters(vdist * (i + 1), lane4_y, 15, 0);

        vehicle_group4.push_back(tp);
    }

    int32_t veh_id = 0;
    for (auto veh : vehicle_group1)
        cfield->AddTrafficParticipant(veh_id++, veh);
    for (auto veh : vehicle_group2)
        cfield->AddTrafficParticipant(veh_id++, veh);
    for (auto veh : vehicle_group3)
        cfield->AddTrafficParticipant(veh_id++, veh);
    for (auto veh : vehicle_group4)
        cfield->AddTrafficParticipant(veh_id++, veh);

    // add lane constraint
    std::shared_ptr<LaneConstraint> lc0 = std::make_shared<LaneConstraint>(fsize_x, fsize_y);
    LanePolylinePoints line0;
    line0.push_back(LanePoint(250, 0));
    line0.push_back(LanePoint(250, 74));
    line0.push_back(LanePoint(213, 148));
    line0.push_back(LanePoint(0, 148));
    lc0->SetLanePolylineKeypoints(line0);

    // std::shared_ptr<LaneConstraint> lc1 = std::make_shared<LaneConstraint>(fsize_x, fsize_y);
    // LanePolylinePoints line1;
    // line1.push_back(LanePoint(287, 0));
    // line1.push_back(LanePoint(287, 74));
    // line1.push_back(LanePoint(499, 74));
    // lc1->SetLanePolylineKeypoints(line1);

    // std::shared_ptr<LaneConstraint> lc2 = std::make_shared<LaneConstraint>(fsize_x, fsize_y);
    // LanePolylinePoints line2;
    // line2.push_back(LanePoint(287, 221));
    // line2.push_back(LanePoint(287, 148));
    // line2.push_back(LanePoint(499, 148));
    // lc2->SetLanePolylineKeypoints(line2);

    std::shared_ptr<LaneConstraint> lc3 = std::make_shared<LaneConstraint>(fsize_x, fsize_y);
    LanePolylinePoints line3;
    line3.push_back(LanePoint(287, 0));
    line3.push_back(LanePoint(287, 221));
    lc3->SetLanePolylineKeypoints(line3);

    cfield->AddLaneConstraints(0, lc0);
    // cfield->AddLaneConstraints(1, lc1);
    // cfield->AddLaneConstraints(2, lc2);
    cfield->AddLaneConstraints(3, lc3);

    // update collision field before use
    cfield->UpdateCollisionField();
    std::cout << "collision field created in " << timer.mtoc() << " seconds" << std::endl;

    timer.tic();

    // perform search on collision field
    FieldTile tile_s(250 + half_lane_width, 0, cfield);
    FieldTile tile_g(0, 148 + half_lane_width, cfield);
    auto path = Dijkstra::IncSearch(tile_s, tile_g, GetNeighbourFunc_t<FieldTile>(GetFieldTileNeighbour(cfield)));
    std::cout << "dijkstra searched for " << timer.mtoc() << " seconds" << std::endl;
    std::cout << "path length: " << path.size() << std::endl;

    // FastPlot::ShowMatrixAsColorMap(cfield->collision_threat_matrix_);
    FastPlot::ShowPathOnColorMap(cfield->collision_threat_matrix_, FieldSearch::GetPathWaypoints(path));

    return 0;
}