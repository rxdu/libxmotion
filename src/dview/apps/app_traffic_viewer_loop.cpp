#include <iostream>

#include "traffic_viewer/traffic_viewer.hpp"

using namespace autodrive;

int main()
{
    TrafficViewer viewer("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    int width = 960;
    int height = width * viewer.aspect_ratio_;

    viewer.SetupViewer(width, height, "Traffic Viewer");
    viewer.Start();

    return 0;
}