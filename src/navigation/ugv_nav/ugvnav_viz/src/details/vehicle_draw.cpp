/* 
 * vehicle_draw.cpp
 * 
 * Created on: Oct 28, 2018 10:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/details/vehicle_draw.hpp"

using namespace librav;

void VehicleDraw::DrawVehicle(Polygon &polygon)
{
    gdraw_.DrawPolygon(polygon, false, CvDrawColors::cyan_color);
    gdraw_.DrawPolygonDirection(polygon);
}

void VehicleDraw::DrawVehicle(std::vector<Polygon> &polygons)
{
    for(auto& poly : polygons)
        DrawVehicle(poly);
}

void VehicleDraw::DrawVehiclePath(std::vector<Polyline> &path, std::vector<Polygon> polygons)
{
}