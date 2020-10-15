/* 
 * point_model.hpp
 * 
 * Created on: Sep 28, 2018 00:10
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POINT_MODEL_HPP
#define POINT_MODEL_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

namespace ivnav
{
// Reference:
//  [1] Broadhurst, A., S. Baker, and T. Kanade. 2005. “Monte Carlo Road Safety Reasoning.”
//      In IEEE Proceedings. Intelligent Vehicles Symposium, 2005., 319–24.
class LongitudinalDynamics
{
public:
  using control_t = double;
  LongitudinalDynamics(control_t u);

  // x1 = s, x2 = v
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

private:
  double u_ = 0.0;
  static constexpr double v_sw = 7.3;
  static constexpr double a_max = 7;
};
} // namespace ivnav

#endif /* POINT_MODEL_HPP */
