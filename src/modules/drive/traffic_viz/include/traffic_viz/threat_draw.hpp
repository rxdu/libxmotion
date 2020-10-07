/* 
 * threat_draw.hpp
 * 
 * Created on: Nov 09, 2018 23:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_DRAW_HPP
#define THREAT_DRAW_HPP

#include "prediction/vehicle_threat.hpp"
#include "prediction/threat_field.hpp"

#include "cvdraw/cvdraw.hpp"

namespace autodrive
{
namespace ThreatViz
{
void DrawCollisionThreat(CvCanvas &canvas, VehicleThreat &threat, int32_t t_k, double cx, double cy, double xspan, double yspan);
void DrawCollisionThreatField(CvCanvas &canvas, ThreatField &field, int32_t t_k, double cx, double cy, double xspan, double yspan);
}; // namespace ThreatViz
} // namespace autodrive

#endif /* THREAT_DRAW_HPP */
