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
#include "threat_field/dynamic_threat_model.hpp"

#include "lightviz/details/cartesian_canvas.hpp"
#include "lightviz/details/geometry_draw.hpp"

namespace librav
{
class VehicleDraw
{
  public:
    VehicleDraw(CartesianCanvas &canvas) : canvas_(canvas), gdraw_(GeometryDraw(canvas_)){};

    void DrawVehicle(Polygon polygon, cv::Scalar ln_color = CvDrawColors::orange_color, int32_t ln_width = 1);
    void DrawVehicle(Polygon polygon, int32_t id, cv::Scalar ln_color = CvDrawColors::orange_color, int32_t ln_width = 1);
    void DrawVehicle(std::vector<Polygon> &polygons, cv::Scalar ln_color = CvDrawColors::orange_color, int32_t ln_width = 1);
    void DrawVehiclePath(std::vector<Polyline> &path, std::vector<Polygon> polygons = {});

    void DrawVehicleStaticCollision(VehicleStaticThreat threat, Polygon polygon);

  private:
    CartesianCanvas &canvas_;
    GeometryDraw gdraw_;
};
} // namespace librav

#endif /* VEHICLE_DRAW_HPP */
