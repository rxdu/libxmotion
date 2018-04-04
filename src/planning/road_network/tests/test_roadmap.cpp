#include <iostream>

#include "road_network/road_map.hpp"

using namespace librav;

int main()
{
    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    map.CreateDenseGrid(5);

    return 0;
}