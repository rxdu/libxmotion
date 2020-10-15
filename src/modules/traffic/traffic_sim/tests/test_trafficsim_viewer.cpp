#include <iostream>

#include "trafficsim_viewer/trafficsim_viewer.hpp"

using namespace ivnav;

int main()
{
    TrafficSimViewer viewer("/home/rdu/Workspace/librav/data/road_map/one_way_merging_horizontal.osm");

    // viewer.SetupViewer();
    viewer.Start();

    return 0;
}