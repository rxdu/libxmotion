/* 
 * roadmap_draw.cpp
 * 
 * Created on: Oct 28, 2018 10:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/roadmap_draw.hpp"

#include "geometry/geometry_draw.hpp"
#include "decomp/curvilinear_grid_draw.hpp"

using namespace ivnav;

void RoadMapViz::DrawLanes(CvCanvas &canvas, std::shared_ptr<RoadMap> map, bool show_center_line)
{
    if (show_center_line)
    {
        for (auto &polyline : map->GetAllLaneCenterPolylines())
            GeometryViz::DrawPolyline(canvas, polyline, false, CvColors::black_color);
    }

    for (auto &polyline : map->GetAllLaneBoundPolylines())
        GeometryViz::DrawPolyline(canvas, polyline, false, CvColors::silver_color);
}

void RoadMapViz::DrawTrafficChannelGrid(CvCanvas &canvas, std::shared_ptr<RoadMap> map, TrafficChannel *channel, bool show_center_line)
{
    if (channel->grid_ != nullptr)
        CurvilinearGridViz::DrawCurvilinearGrid(canvas, *channel->grid_.get());

    if (show_center_line)
    {
        GeometryViz::DrawParametricCurve(canvas, channel->center_curve_, 0.1, CvColors::black_color);
        GeometryViz::DrawPolyline(canvas, channel->center_line_, true, CvColors::black_color);
    }
}

void RoadMapViz::DrawTrafficChannelCenterline(CvCanvas &canvas, std::shared_ptr<RoadMap> map, TrafficChannel *channel)
{
    GeometryViz::DrawPolyline(canvas, channel->center_line_, true, CvColors::black_color);
}