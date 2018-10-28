/* 
 * polygon_viz.cpp
 * 
 * Created on: Aug 10, 2018 10:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/polygon_viz.hpp"

#include <cmath>

#include "lightviz/details/geometric_draw.hpp"

using namespace librav;
using namespace LightViz;
using namespace CvDraw;

void LightViz::ShowPolyline(const Polyline &polyline, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    double xspan = polyline.GetMaxX() - polyline.GetMinX();
    double yspan = polyline.GetMaxY() - polyline.GetMinY();

    double xmin = polyline.GetMinX() - xspan * 0.2;
    double xmax = polyline.GetMaxX() + xspan * 0.2;
    double ymin = polyline.GetMinY() - yspan * 0.2;
    double ymax = polyline.GetMaxY() + yspan * 0.2;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);
    canvas = gdraw.DrawPolyline(canvas, polyline);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPolyline(const std::vector<Polyline> &polylines, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : polylines)
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polyline : polylines)
        canvas = gdraw.DrawPolyline(canvas, polyline);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPolylinePosition(const std::vector<Polyline> &polylines, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : polylines)
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    std::vector<SimplePoint> points;
    for (auto &polyline : polylines)
    {
        canvas = gdraw.DrawPolyline(canvas, polyline);
        auto new_pts = polyline.GetSimplePoints();
        // points.insert(points.end(), new_pts.begin(), new_pts.end());
        points.push_back(new_pts.front());
        points.push_back(new_pts.back());
    }
    gdraw.WritePointPosition(canvas, points);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPolygon(const Polygon &polygon, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    double xspan = polygon.GetMaxX() - polygon.GetMinX();
    double yspan = polygon.GetMaxY() - polygon.GetMinY();

    double xmin = polygon.GetMinX() - xspan * 0.2;
    double xmax = polygon.GetMaxX() + xspan * 0.2;
    double ymin = polygon.GetMinY() - yspan * 0.2;
    double ymax = polygon.GetMaxY() + yspan * 0.2;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);
    canvas = gdraw.DrawPolygon(canvas, polygon);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPolygon(const std::vector<Polygon> &polygons, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polygon : polygons)
    {
        xmins.push_back(polygon.GetMinX());
        xmaxs.push_back(polygon.GetMaxX());
        ymins.push_back(polygon.GetMinY());
        ymaxs.push_back(polygon.GetMaxY());
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polygon : polygons)
        canvas = gdraw.DrawPolygon(canvas, polygon);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowFilledPolygon(const std::vector<Polygon> &polygons, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polygon : polygons)
    {
        xmins.push_back(polygon.GetMinX());
        xmaxs.push_back(polygon.GetMaxX());
        ymins.push_back(polygon.GetMinY());
        ymaxs.push_back(polygon.GetMaxY());
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polygon : polygons)
        canvas = gdraw.DrawFilledPolygon(canvas, polygon);
    for (auto &polygon : polygons)
        canvas = gdraw.DrawPolygon(canvas, polygon);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowFilledPolygon(const std::vector<Polygon> &polygons, const std::vector<Polygon> &bound_polygons, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polygon : polygons)
    {
        xmins.push_back(polygon.GetMinX());
        xmaxs.push_back(polygon.GetMaxX());
        ymins.push_back(polygon.GetMinY());
        ymaxs.push_back(polygon.GetMaxY());
    }
    for (auto &polygon : bound_polygons)
    {
        xmins.push_back(polygon.GetMinX());
        xmaxs.push_back(polygon.GetMaxX());
        ymins.push_back(polygon.GetMinY());
        ymaxs.push_back(polygon.GetMaxY());
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polygon : polygons)
        canvas = gdraw.DrawFilledPolygon(canvas, polygon);
    for (auto &polygon : bound_polygons)
        canvas = gdraw.DrawPolygon(canvas, polygon);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowLanePolylines(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers, int32_t pixel_per_unit, std::string window_name, bool save_img)
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, CvDrawColors::silver_color);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPathInLane(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers, std::vector<Polyline> &path, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
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

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, CvDrawColors::silver_color);

    for (auto &polyline : path)
        canvas = gdraw.DrawPolyline(canvas, polyline, show_wp, CvDrawColors::lime_color, 2);

    ShowImage(canvas, window_name, save_img);
}