/* 
 * vehicle_draw.hpp
 * 
 * Created on: Oct 28, 2018 10:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_DRAW_HPP
#define VEHICLE_DRAW_HPP

#include "geometry/polyline.hpp"
#include "geometry/polygon.hpp"
#include "prediction/dynamic_threat_model.hpp"

#include "cvdraw/cvdraw.hpp"

namespace autodrive
{
namespace VehicleViz
{
void DrawVehicle(CvCanvas &canvas, Polygon polygon, cv::Scalar ln_color = CvColors::orange_color, int32_t ln_width = 1);
void DrawVehicle(CvCanvas &canvas, Polygon polygon, int32_t id, cv::Scalar ln_color = CvColors::orange_color, int32_t ln_width = 1);
void DrawVehicle(CvCanvas &canvas, std::vector<Polygon> &polygons, cv::Scalar ln_color = CvColors::orange_color, int32_t ln_width = 1);
void DrawVehiclePath(CvCanvas &canvas, std::vector<Polyline> &path, std::vector<Polygon> polygons = {});

void DrawVehicleStaticCollision(CvCanvas &canvas, VehicleStaticThreat threat, Polygon polygon);
}; // namespace VehicleViz
} // namespace autodrive

#endif /* VEHICLE_DRAW_HPP */
