/* 
 * roadmap_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/roadmap_viz.hpp"

#include "lightviz/details/geometry_draw.hpp"
#include "ugvnav_viz/details/roadmap_draw.hpp"
#include "ugvnav_viz/details/vehicle_draw.hpp"

using namespace librav;
using namespace CvDraw;

RoadMapViz::RoadMapViz(std::shared_ptr<RoadMap> map, int32_t ppu) : road_map_(map), ppu_(ppu)
{
    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : map->GetAllLaneBoundPolylines())
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    for (auto &polyline : map->GetAllLaneCenterPolylines())
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    double bd_xl = *std::min_element(xmins.begin(), xmins.end());
    double bd_xr = *std::max_element(xmaxs.begin(), xmaxs.end());
    double bd_yb = *std::min_element(ymins.begin(), ymins.end());
    double bd_yt = *std::max_element(ymaxs.begin(), ymaxs.end());

    double xspan = bd_xr - bd_xl;
    double yspan = bd_yt - bd_yb;

    xmin_ = bd_xl - xspan * 0.1;
    xmax_ = bd_xr + xspan * 0.1;
    ymin_ = bd_yb - yspan * 0.1;
    ymax_ = bd_yt + yspan * 0.1;
}

RoadMapViz &RoadMapViz::GetInstance(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit)
{
    static RoadMapViz viz(map, pixel_per_unit);
    return viz;
}

void RoadMapViz::SetupRoadMapViz(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit)
{
    RoadMapViz &viz = RoadMapViz::GetInstance(map, pixel_per_unit);
}

CartesianCanvas RoadMapViz::CreateCanvas(bool use_jetcolor)
{
    CartesianCanvas canvas(ppu_);
    
    if (use_jetcolor)
        canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_, CvDrawColors::jet_colormap_lowest);
    else
        canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_);

    return canvas;
}

void RoadMapViz::ShowLanes(bool show_center_line, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);

    road_draw.DrawLanes(show_center_line);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void RoadMapViz::ShowTrafficChannel(TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);

    road_draw.DrawLanes(true);
    road_draw.DrawTrafficChannelGrid(channel, false);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void RoadMapViz::ShowTrafficChannelCenterline(TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);

    road_draw.DrawLanes(true);
    road_draw.DrawTrafficChannelCenterline(channel);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void RoadMapViz::ShowVehicle(Polygon polygon, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(polygon);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void RoadMapViz::ShowVehicle(std::vector<Polygon> &polygons, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(polygons); 

    ShowImage(canvas.paint_area, window_name, save_img);
}