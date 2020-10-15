/* 
 * roadmap_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/roadmap_viz.hpp"

#include "traffic_viz/roadmap_draw.hpp"
#include "traffic_viz/vehicle_draw.hpp"

namespace ivnav
{
CvCanvas UGVNavViz::CreateCanvas(std::shared_ptr<RoadMap> map, int32_t ppu, bool use_jetcolor)
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

    double xmin = bd_xl - xspan * 0.05;
    double xmax = bd_xr + xspan * 0.05;
    double ymin = bd_yb - yspan * 0.05;
    double ymax = bd_yt + yspan * 0.05;

    if (use_jetcolor)
    {
        CvCanvas canvas(ppu, CvColors::jet_colormap_lowest);
        canvas.Resize(xmin, xmax, ymin, ymax);
        return canvas;
    }
    else
    {
        CvCanvas canvas(ppu);
        canvas.Resize(xmin, xmax, ymin, ymax);
        return canvas;
    }
}

void UGVNavViz::ShowLanes(std::shared_ptr<RoadMap> map, bool show_center_line, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, show_center_line);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowTrafficChannel(std::shared_ptr<RoadMap> map, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowTrafficChannelCenterline(std::shared_ptr<RoadMap> map, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelCenterline(canvas, map, channel);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowVehicleOnMap(std::shared_ptr<RoadMap> map, Polygon polygon, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, true);
    VehicleViz::DrawVehicle(canvas, polygon);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowVehicleOnMap(std::shared_ptr<RoadMap> map, std::vector<Polygon> &polygons, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, true);
    VehicleViz::DrawVehicle(canvas, polygons);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}
} // namespace ivnav