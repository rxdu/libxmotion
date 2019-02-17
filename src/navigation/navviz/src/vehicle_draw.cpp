/* 
 * vehicle_draw.cpp
 * 
 * Created on: Oct 28, 2018 10:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "navviz/vehicle_draw.hpp"
#include "geometry/simple_point.hpp"
#include "coreviz/geometry_draw.hpp"

namespace librav
{
void VehicleViz::DrawVehicle(CvCanvas &canvas, Polygon polygon, cv::Scalar ln_color, int32_t ln_width)
{
    GeometryViz::DrawPolygon(canvas, polygon, false, ln_color, ln_width);
    GeometryViz::DrawPolygonDirection(canvas, polygon, ln_color, ln_width);
}

void VehicleViz::DrawVehicle(CvCanvas &canvas, Polygon polygon, int32_t id, cv::Scalar ln_color, int32_t ln_width)
{
    GeometryViz::DrawPolygon(canvas, polygon, false, ln_color, ln_width);
    GeometryViz::DrawPolygonDirection(canvas, polygon, ln_color, ln_width);
    SimplePoint pt;
    for (int i = 0; i < 4; ++i)
    {
        pt.x += polygon.GetPoint(i).x;
        pt.y += polygon.GetPoint(i).y;
    }
    pt.x = pt.x / 4.0;
    pt.y = pt.y / 4.0;
    // GeometryViz::WriteTextAtPosition(canvas, std::to_string(id), pt);
    canvas.WriteText(std::to_string(id), {pt.x, pt.y}, 1, CvColors::cyan_color);
}

void VehicleViz::DrawVehicle(CvCanvas &canvas, std::vector<Polygon> &polygons, cv::Scalar ln_color, int32_t ln_width)
{
    for (auto &poly : polygons)
        DrawVehicle(canvas, poly);
}

void VehicleViz::DrawVehiclePath(CvCanvas &canvas, std::vector<Polyline> &path, std::vector<Polygon> polygons)
{
}

void VehicleViz::DrawVehicleStaticCollision(CvCanvas &canvas, VehicleStaticThreat threat, Polygon polygon)
{
    GeometryViz::DrawDistribution(canvas, threat.pose.position.x, threat.pose.position.y, 20, 20,
                                  std::function<double(double, double)>(threat));

    GeometryViz::DrawPolygon(canvas, polygon, false, CvColors::black_color, 2);
    GeometryViz::DrawPolygonDirection(canvas, polygon, CvColors::orange_color, 2);
}
} // namespace librav