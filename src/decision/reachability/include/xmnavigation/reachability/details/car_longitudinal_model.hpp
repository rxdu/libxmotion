/*
 * car_longitudinal_model.hpp
 *
 * Created on: Oct 30, 2018 05:47
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAR_LONGITUDINAL_MODEL_HPP
#define CAR_LONGITUDINAL_MODEL_HPP

#include <boost/numeric/odeint.hpp>

namespace xmotion {
// Reference:
//  [1] Althoff, M., and A. Mergel. 2011. “Comparison of Markov Chain
//  Abstraction
//      and Monte Carlo Simulation for the Safety Assessment of Autonomous
//      Cars.” IEEE Transactions on Intelligent Transportation Systems 12 (4):
//      1237–47.
class CarLongitudinalModel {
 public:
  using control_type = double;
  using state_type = std::vector<double>;

  CarLongitudinalModel(control_type u) : u_(u) {}

  static constexpr double v_sw = 7.3;    // switching velocity
  static constexpr double v_max = 18.0;  // 18 m/s ~= 40 mph
  static constexpr double a_max = 7.0;   // 7.0 m/s^2

  // x1 = s, x2 = v
  void operator()(const state_type &x, state_type &xd, const double);

 private:
  control_type u_ = 0;
};
}  // namespace xmotion
#endif /* CAR_LONGITUDINAL_MODEL_HPP */
