/* 
 * field_viz.cpp
 * 
 * Created on: Aug 12, 2018 23:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/field_viz_ext.hpp"

#include <cmath>

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;

void LightViz::ShowTrafficParticipant(std::shared_ptr<TrafficParticipant> participant, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(0, 80, 0, 60, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, participant->position_x, participant->position_y, 20, 20, std::function<double(double, double)>(participant->threat_func));

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowCollisionField(std::shared_ptr<CollisionField> cfield, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(cfield->xmin_, cfield->xmax_, cfield->ymin_, cfield->ymax_, LVColors::jet_colormap_lowest);

    std::cout << "params: " << cfield->GetMeanX() << " , " << cfield->GetMeanY() << " , " << cfield->GetSpanX() << " , " << cfield->GetSpanY() << std::endl;

    canvas = gdraw.DrawDistribution(canvas, cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowCollisionFieldInRoadMap(std::shared_ptr<CollisionField> cfield, std::shared_ptr<RoadMap> map, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    auto bounds = map->GetAllLaneBoundPolylines();
    auto centers = map->GetAllLaneCenterPolylines();

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

    double xmin = bd_xl - xspan * 0.1;
    double xmax = bd_xr + xspan * 0.1;
    double ymin = bd_yb - yspan * 0.1;
    double ymax = bd_yt + yspan * 0.1;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    // for (auto &polyline : path)
    //     canvas = gdraw.DrawPolyline(canvas, polyline, show_wp, LVColors::lime_color, 2);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPathSegmentCollisionFieldWithRoadMap(const Polyline &path_seg, std::shared_ptr<CollisionField> cfield, std::shared_ptr<RoadMap> map,
                                                        bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    auto bounds = map->GetAllLaneBoundPolylines();
    auto centers = map->GetAllLaneCenterPolylines();

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

    double xmin = bd_xl - xspan * 0.1;
    double xmax = bd_xr + xspan * 0.1;
    double ymin = bd_yb - yspan * 0.1;
    double ymax = bd_yt + yspan * 0.1;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    canvas = gdraw.DrawPolyline(canvas, path_seg, show_wp, LVColors::lime_color, 2);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowPathCollisionFieldWithRoadMap(const Polyline &path_seg, const std::vector<Polyline> &path, std::shared_ptr<CollisionField> cfield, std::shared_ptr<RoadMap> map,
                                                 bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    auto bounds = map->GetAllLaneBoundPolylines();
    auto centers = map->GetAllLaneCenterPolylines();

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

    double xmin = bd_xl - xspan * 0.1;
    double xmax = bd_xr + xspan * 0.1;
    double ymin = bd_yb - yspan * 0.1;
    double ymax = bd_yt + yspan * 0.1;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax, LVColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    for (auto &polyline : bounds)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::silver_color);

    for (auto &polyline : centers)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::black_color);

    for (auto &polyline : path)
        canvas = gdraw.DrawPolyline(canvas, polyline, false, LVColors::green_color, 2);

    canvas = gdraw.DrawPolyline(canvas, path_seg, show_wp, LVColors::lime_color, 2);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowVehicleFootprint(const Polygon &polygon, double x, double y, double theta, int32_t pixel_per_unit, std::string window_name, bool save_img)
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
