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

#include "lightviz/details/cartesian_canvas.hpp"

#include "threat_field/dynamic_threat_model.hpp"
#include "threat_field/threat_field.hpp"

namespace librav
{
class ThreatDraw
{
  public:
    ThreatDraw(CartesianCanvas &canvas) : canvas_(canvas){};

    void DrawCollisionThreat(DynamicThreatModel& threat,double cx, double cy, double xspan, double yspan);
    void DrawCollisionThreatField(ThreatField& field,double cx, double cy, double xspan, double yspan);

  private:
    CartesianCanvas &canvas_;
};
} // namespace librav

#endif /* THREAT_DRAW_HPP */
