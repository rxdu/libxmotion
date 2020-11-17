/*
 * robot.hpp
 *
 * Created on: Oct 23, 2020 21:06
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "odeint.hpp"

namespace rnav {
template <typename Model>
class Robot {
 public:
  using State = typename Model::state_type;
  using Command = typename Model::control_type;

 public:
  State StepForward(State x0, Command u, double t0, double tf, double dt) {
    State x = x0;
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<typename Model::state_type>(),
        Model(u), x, t0, tf, dt);
    return x;
  }
};
}  // namespace rnav

#endif /* ROBOT_HPP */
