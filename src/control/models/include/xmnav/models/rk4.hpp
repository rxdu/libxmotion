/*
 * @file rk4.hpp
 * @brief Fixed-step classical Runge-Kutta 4 propagation.
 *
 * Replaces the boost::numeric::odeint runge_kutta4/integrate_const usage
 * of the retired model/SystemPropagator with ~20 dependency-free lines:
 * same method, same fixed step (the final step is shortened to land
 * exactly on tf). For models exposing State Deriv(state, control).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_RK4_HPP
#define XMNAV_MODELS_RK4_HPP

#include <algorithm>

namespace xmotion {

template <typename Model>
typename Model::State Rk4Propagate(const Model &model,
                                   typename Model::State x,
                                   const typename Model::Control &u,
                                   double t0, double tf, double dt) {
  using State = typename Model::State;
  double t = t0;
  while (t < tf - 1e-12) {
    const double h = std::min(dt, tf - t);
    const State k1 = model.Deriv(x, u);
    const State k2 = model.Deriv(x + 0.5 * h * k1, u);
    const State k3 = model.Deriv(x + 0.5 * h * k2, u);
    const State k4 = model.Deriv(x + h * k3, u);
    x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    t += h;
  }
  return x;
}

}  // namespace xmotion

#endif  // XMNAV_MODELS_RK4_HPP
