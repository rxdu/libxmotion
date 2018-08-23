/* 
 * roadmap_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/roadmap_viz.hpp"

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;

RoadMapViz::RoadMapViz()
{
    xmin_ = 0.0;
    xmax_ = 1.0;
    ymin_ = 0.0;
    ymax_ = 1.0;
}

RoadMapViz &RoadMapViz::GetInstance()
{
    static RoadMapViz viz;
    return viz;
}

void RoadMapViz::SetupRoadMapViz(std::shared_ptr<RoadMap> map)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    viz.road_map_ = map;
    viz.boundary_lines_ = map->GetAllLaneBoundPolylines();
    viz.center_lines_ = map->GetAllLaneCenterPolylines();

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : viz.boundary_lines_)
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    for (auto &polyline : viz.center_lines_)
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

    viz.xmin_ = bd_xl - xspan * 0.1;
    viz.xmax_ = bd_xr + xspan * 0.1;
    viz.ymin_ = bd_yb - yspan * 0.1;
    viz.ymax_ = bd_yt + yspan * 0.1;
}

void RoadMapViz::ShowLanes(bool show_center_line, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(viz.xmin_, viz.xmax_, viz.ymin_, viz.ymax_, LVColors::jet_colormap_lowest);

    for (auto &polyline : viz.boundary_lines_)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    if (show_center_line)
    {
        for (auto &polyline : viz.center_lines_)
            canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);
    }

    LightViz::ShowImage(canvas, window_name, save_img);
}

void RoadMapViz::ShowVehicle(Polygon &polygon, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(viz.xmin_, viz.xmax_, viz.ymin_, viz.ymax_, LVColors::jet_colormap_lowest);

    for (auto &polyline : viz.boundary_lines_)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : viz.center_lines_)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    canvas = gdraw.DrawPolygon(canvas, polygon, false, LVColors::cyan_color);
    canvas = gdraw.DrawPolygonDirection(canvas, polygon);

    LightViz::ShowImage(canvas, window_name, save_img);
}

void RoadMapViz::ShowVehicleFootprints(std::vector<Polygon> &polygons, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(viz.xmin_, viz.xmax_, viz.ymin_, viz.ymax_, LVColors::jet_colormap_lowest);

    for (auto &polyline : viz.boundary_lines_)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : viz.center_lines_)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    for (auto &polygon : polygons)
    {
        canvas = gdraw.DrawPolygon(canvas, polygon, false, LVColors::cyan_color);
        canvas = gdraw.DrawPolygonDirection(canvas, polygon, LVColors::red_color, 2);
    }

    LightViz::ShowImage(canvas, window_name, save_img);
}