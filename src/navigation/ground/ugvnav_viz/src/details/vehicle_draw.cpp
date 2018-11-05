/* 
 * vehicle_draw.cpp
 * 
 * Created on: Oct 28, 2018 10:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/details/vehicle_draw.hpp"
#include "geometry/simple_point.hpp"

using namespace librav;

void VehicleDraw::DrawVehicle(Polygon polygon)
{
    gdraw_.DrawPolygon(polygon, false, CvDrawColors::orange_color, 2);
    gdraw_.DrawPolygonDirection(polygon, CvDrawColors::orange_color, 2);
}

void VehicleDraw::DrawVehicle(Polygon polygon, int32_t id)
{
    gdraw_.DrawPolygon(polygon, false, CvDrawColors::orange_color, 2);
    gdraw_.DrawPolygonDirection(polygon, CvDrawColors::orange_color, 2);
    SimplePoint pt;
    for (int i = 0; i < 4; ++i)
    {
        pt.x += polygon.GetPoint(i).x;
        pt.y += polygon.GetPoint(i).y;
    }
    pt.x = pt.x / 4.0;
    pt.y = pt.y / 4.0;
    gdraw_.WriteTextAtPosition(std::to_string(id), pt);
}

void VehicleDraw::DrawVehicle(std::vector<Polygon> &polygons)
{
    for (auto &poly : polygons)
        DrawVehicle(poly);
}

void VehicleDraw::DrawVehiclePath(std::vector<Polyline> &path, std::vector<Polygon> polygons)
{
}

void VehicleDraw::DrawVehicleStaticCollision(VehicleStaticThreat threat, Polygon polygon)
{
    gdraw_.DrawDistribution(threat.pose.position.x, threat.pose.position.y, 20, 20,
                            std::function<double(double, double)>(threat));
                            
    gdraw_.DrawPolygon(polygon, false, CvDrawColors::black_color, 2);
    gdraw_.DrawPolygonDirection(polygon, CvDrawColors::orange_color, 2);
}
