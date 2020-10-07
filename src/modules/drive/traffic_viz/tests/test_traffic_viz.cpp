#include <iostream>
#include <cmath>

#include "traffic_map/map_loader.hpp"

#include "lightviz/navviz.hpp"

using namespace autodrive;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    ////////////////////////////////////////////////////////////////////////

    TrafficViz::SetupTrafficViz(loader.road_map, 10);
    TrafficViz::ShowLanes();

    TrafficViz::ShowLanes();

    Polygon fp;
    fp.AddPoint(1.2 * 2, 0.9);
    fp.AddPoint(1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, 0.9);

    std::vector<Polygon> polys;
    auto fp_start = fp.TransformRT(57, 36, 82 / 180.0 * M_PI);
    auto fp_final = fp.TransformRT(4, 68, 170.0 / 180.0 * M_PI);
    polys.push_back(fp_start);
    polys.push_back(fp_final);

    TrafficViz::ShowVehicle(fp);
    TrafficViz::ShowVehicle(polys);

    return 0;
}