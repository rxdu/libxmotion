#include <iostream>

#include "traffic_viewer/traffic_viewer.hpp"

using namespace librav;

int main()
{
    TrafficViewer viewer("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    viewer.SetupViewer();
    viewer.Start();

    return 0;
}