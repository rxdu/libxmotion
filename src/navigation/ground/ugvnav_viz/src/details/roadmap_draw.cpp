/* 
 * roadmap_draw.cpp
 * 
 * Created on: Oct 28, 2018 10:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/details/roadmap_draw.hpp"

using namespace librav;

RoadMapDraw::RoadMapDraw(std::shared_ptr<RoadMap> map, CartesianCanvas canvas) : canvas_(canvas), gdraw_(canvas_), cdraw_(canvas_), road_map_(map)
{
    boundary_lines_ = map->GetAllLaneBoundPolylines();
    center_lines_ = map->GetAllLaneCenterPolylines();
}

void RoadMapDraw::DrawLanes(bool show_center_line)
{
    for (auto &polyline : boundary_lines_)
        gdraw_.DrawPolyline(polyline, false, CvDrawColors::silver_color);

    if (show_center_line)
    {
        for (auto &polyline : center_lines_)
            gdraw_.DrawPolyline(polyline, false, CvDrawColors::black_color);
    }
}

void RoadMapDraw::DrawTrafficChannelGrid(TrafficChannel &channel, bool show_center_line)
{
    if (channel.grid_ != nullptr)
        cdraw_.DrawCurvilinearGrid(*channel.grid_.get());

    if (show_center_line)
    {
        gdraw_.DrawParametricCurve(channel.center_curve_, 0.1, CvDrawColors::black_color);
        gdraw_.DrawPolyline(channel.center_line_, true, CvDrawColors::black_color);
    }
}

void RoadMapDraw::DrawTrafficChannelCenterline(TrafficChannel &channel)
{
    gdraw_.DrawPolyline(channel.center_line_, true, CvDrawColors::black_color);
}