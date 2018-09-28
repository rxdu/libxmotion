/* 
 * bicycle_model.hpp
 * 
 * Created on: Mar 20, 2018 17:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

namespace librav
{
// Reference:
//  [1] B. Paden, M. Cap, S. Z. Yong, D. Yershov, and E. Frazzoli, “A Survey of
//      Motion Planning and Control Techniques for Self-driving Urban Vehicles,”
//      Apr. 2016.
//  [2] J. Kong, M. Pfeiffer, G. Schildbach, and F. Borrelli, “Kinematic and dynamic
//      vehicle models for autonomous driving control design,” in 2015 IEEE
//      Intelligent Vehicles Symposium (IV), Jun. 2015, pp. 1094–1099.
//  [3] R. Rajamani, Vehicle Dynamics and Control. Springer Science & Business
//      Media, Dec. 2011.

/*
 * Bicycle kinematics model (rear wheel):
 *  dot_x = v(t) * cos(theta(t)) 
 *  dot_y = v(t) * sin(theta(t))
 *  dot_v = a(t) 
 *  dot_theta = v(t)/L * tan(delta(t))
 * State: (x, y, v, theta)
 * Control input: (a, delta) - acceleration, steering angle of front wheel
 */
class BicycleKinematics
{
public:
  struct CtrlInput
  {
    CtrlInput() : a(0.0), delta(0.0) {}
    CtrlInput(double acc, double ster) : a(acc), delta(ster) {}

    double a;
    double delta;
  };

  using control_t = CtrlInput;

  /////////////////////////////////////

  BicycleKinematics(control_t u);

  // x1 = x, x2 = y, x3 = v, x4 = theta
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

private:
  control_t u_ = {0.0, 0.0};
  static constexpr double L = 2.4;
};
} // namespace librav

#endif /* BICYCLE_MODEL_HPP */
