#include <iostream>

#include "traffic_viewer/traffic_viewer.hpp"

using namespace autodrive;

int main()
{
    TrafficViewer viewer("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    int width = 1080;
    int height = width * viewer.aspect_ratio_;

    viewer.SetupViewer(width, height, "Traffic Viewer");
    viewer.Start();

    return 0;
}