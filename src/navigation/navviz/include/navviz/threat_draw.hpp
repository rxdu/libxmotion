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

#include "threat_field/vehicle_threat.hpp"
#include "threat_field/threat_field.hpp"

#include "cvdraw/cvdraw.hpp"

namespace librav
{
namespace ThreatViz
{
void DrawCollisionThreat(CvCanvas &canvas, VehicleThreat &threat, int32_t t_k, double cx, double cy, double xspan, double yspan);
void DrawCollisionThreatField(CvCanvas &canvas, ThreatField &field, int32_t t_k, double cx, double cy, double xspan, double yspan);
}; // namespace ThreatViz
} // namespace librav

#endif /* THREAT_DRAW_HPP */
