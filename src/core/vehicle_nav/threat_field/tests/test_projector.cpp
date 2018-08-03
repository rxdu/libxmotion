#include <iostream>
#include <memory>

#include "threat_field/threat_projector.hpp"
#include "road_network/road_map.hpp"
#include "traffic_flow/road_cost_map.hpp"
#include "traffic_flow/traffic_flow_map.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm", 10);
    std::cout << "loaded map in " << timer.toc() << " seconds" << std::endl;

    // LightViz::ShowMatrixAsColorMap(map->GetFullDrivableAreaGrid()->GetGridMatrix(true), "full drivable area", true);

    std::shared_ptr<TrafficFlowMap> tf = std::make_shared<TrafficFlowMap>(map);
    tf->BuildRoadGrid(10);
    // LightViz::ShowSquareGrid(tf->road_grid_.get(), 100, "raw grid", true);

    tf->AddTrafficFlowSource("s2", GridCoordinate(120, 18));
    tf->AddTrafficFlowSource("s4", GridCoordinate(51, 67));
    tf->AddTrafficFlowSource("s6", GridCoordinate(4, 7));

    tf->AddTrafficFlowSink("s1");
    tf->AddTrafficFlowSink("s3");
    tf->AddTrafficFlowSink("s5");

    tf->IdentifyTrafficFlow();

    ThreatProjector projector(map);
    projector.SetRouteStartGoal("s4", "s1");

    LightViz::ShowMatrixAsColorMap(projector.GetRouteDrivableMask(true), "driving", false);

    return 0;
}