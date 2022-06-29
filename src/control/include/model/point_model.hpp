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

#include <vector>

namespace robosw {
// Reference:
//  [1] Broadhurst, A., S. Baker, and T. Kanade. 2005. “Monte Carlo Road Safety
//  Reasoning.”
//      In IEEE Proceedings. Intelligent Vehicles Symposium, 2005., 319–24.
class LongitudinalDynamics {
 public:
  using control_type = double;
  using state_type = std::vector<double>;

  LongitudinalDynamics(control_type u) : u_(u){};

  // x1 = s, x2 = v
  void operator()(const state_type &x, state_type &xd, const double) {
    xd[0] = x[1];

    if (x[1] <= 0)
      xd[1] = 0;
    else if (x[1] > v_sw && u_ > 0)
      xd[1] = a_max * v_sw / x[1] * u_;
    else
      xd[1] = a_max * u_;
  }

 private:
  double u_ = 0.0;
  static constexpr double v_sw = 7.3;
  static constexpr double a_max = 7;
};
}  // namespace robosw

#endif /* POINT_MODEL_HPP */
