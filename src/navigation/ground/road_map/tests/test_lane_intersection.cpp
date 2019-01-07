#include <iostream>

#include "road_map/road_map.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    stopwatch::StopWatch timer;

    RoadMap map("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map.MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map.PrintInfo();

    // map.GenerateMapMasks(15);
    std::cout << "map loaded in " << timer.toc() << " seconds" << std::endl;

    map.CheckLaneletCollision();

    // Polygon p1 = map.GetLanePolygon("s3");
    // // Polygon p2 = map.GetLanePolygon("icm2");
    // Polygon p2 = map.GetLanePolygon("s1");
    // std::cout << "collision: " << Collision::Check(p1, p2) << std::endl;

    return 0;
}