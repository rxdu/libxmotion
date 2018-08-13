/* 
 * field_viz.cpp
 * 
 * Created on: Aug 11, 2018 11:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/field_viz.hpp"

#include <cmath>

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;
using namespace LightViz;

void LightViz::ShowFieldDistribution(double cx, double cy, std::function<double(double, double)> dist_fun,
                         bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(0, 80, 0, 60, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, cx, cy, 20, 20, dist_fun);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPathLaneInField(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers,
                                   std::vector<Polyline> &path,
                                   double cx, double cy, std::function<double(double, double)> dist_fun,
                                   bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : bounds)
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    for (auto &polyline : centers)
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

    double xmin = bd_xl - xspan * 0.2;
    double xmax = bd_xr + xspan * 0.2;
    double ymin = bd_yb - yspan * 0.2;
    double ymax = bd_yt + yspan * 0.2;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, cx, cy, 20, 20, dist_fun);

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    for (auto &polyline : path)
        canvas = gdraw.DrawPolyline(canvas, polyline, show_wp, LVColors::lime_color, 2);

    ShowImage(canvas, window_name, save_img);
}