#include <iostream>
#include <cmath>

#include "road_map/road_map.hpp"

#include "lightviz/roadmap_viz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    ////////////////////////////////////////////////////////////////////////

    // RoadMapViz::SetupRoadMapViz(map, 10);
    // auto canvas = UGVNavViz::CreateCanvas(map);
    // RoadMapViz::ShowLanes(canvas, map);

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

    // RoadMapViz::ShowVehicleOnMap(canvas, map, fp);
    UGVNavViz::ShowVehicleOnMap(map, polys);

    return 0;
}