/* 
 * traffic_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/traffic_viz.hpp"

#include "lightviz/details/geometry_draw.hpp"
#include "ugvnav_viz/details/roadmap_draw.hpp"
#include "ugvnav_viz/details/vehicle_draw.hpp"
#include "ugvnav_viz/details/lattice_draw.hpp"
#include "ugvnav_viz/roadmap_viz.hpp"

using namespace librav;
using namespace CvDraw;

void TrafficViz::SetupTrafficViz(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit)
{
    RoadMapViz::SetupRoadMapViz(map, pixel_per_unit);
}

void TrafficViz::ShowLanes(bool show_center_line, std::string window_name, bool save_img)
{
    RoadMapViz::ShowLanes(show_center_line, window_name, save_img);
}

void TrafficViz::ShowTrafficChannel(TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz::ShowTrafficChannel(channel, window_name, save_img);
}

void TrafficViz::ShowVehicle(Polygon polygon, std::string window_name, bool save_img)
{
    RoadMapViz::ShowVehicle(polygon, window_name, save_img);
}

void TrafficViz::ShowVehicle(std::vector<Polygon> &polygons, std::string window_name, bool save_img)
{
    RoadMapViz::ShowVehicle(polygons, window_name, save_img);
}

void TrafficViz::ShowLatticeInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    road_draw.DrawLanes(true);
    road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(lattice);

    ShowImage(canvas.paint_area, window_name, save_img);
}