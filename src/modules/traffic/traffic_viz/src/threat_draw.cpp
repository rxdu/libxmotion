/* 
 * threat_draw.cpp
 * 
 * Created on: Nov 09, 2018 23:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/threat_draw.hpp"
#include "geometry/geometry_draw.hpp"
// #include <tbb/tbb.h>

namespace ivnav
{
void ThreatViz::DrawCollisionThreat(CvCanvas &canvas, VehicleThreat &threat, int32_t t_k, double cx, double cy, double xspan, double yspan)
{
    GeometryViz::DrawDistribution(canvas, cx, cy, xspan, yspan, std::bind(threat, std::placeholders::_1, std::placeholders::_2, t_k));
}

void ThreatViz::DrawCollisionThreatField(CvCanvas &canvas, ThreatField &field, int32_t t_k, double cx, double cy, double xspan, double yspan)
{
    GeometryViz::DrawDistribution(canvas, cx, cy, xspan, yspan, std::bind(field, std::placeholders::_1, std::placeholders::_2, t_k));
}
} // namespace ivnav