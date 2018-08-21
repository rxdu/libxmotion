#include <iostream>
#include <cmath>

#include "road_map/road_map.hpp"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    ////////////////////////////////////////////////////////////////////////

    RoadMapViz::SetupRoadMapViz(map);
    // RoadMapViz::ShowLanes();

    Polygon fp;
    // fp.AddPoint(0.9, 1.2 * 2);
    // fp.AddPoint(-0.9, 1.2 * 2);
    // fp.AddPoint(-0.9, -1.2 * 2);
    // fp.AddPoint(0.9, -1.2 * 2);
    fp.AddPoint(1.2 * 2, 0.9);
    fp.AddPoint(1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, 0.9);
    auto new_fp = fp.TransformRT(57, 36, 82 / 180.0 * M_PI);
    // auto new_fp = fp.TransformRT(4, 68, 170.0 / 180.0 * M_PI);

    RoadMapViz::ShowVehicle(new_fp);

    // LightViz::ShowPolygon(map.GetAllLanePolygons(), 10);
    // LightViz::ShowPolyline(map.GetAllLaneBoundPolylines(), 10);
    // LightViz::ShowLanePolylines(map.GetAllLaneBoundPolylines(), map.GetAllLaneCenterPolylines());
    // LightViz::ShowPolylinePosition(map.GetAllLaneCenterPolylines(), 10);

    // std::vector<Polygon> roi;
    // roi.push_back(map.GetLanePolygon("s1"));
    // LightViz::ShowFilledPolygon(roi, map.GetAllLanePolygons());

    return 0;
}