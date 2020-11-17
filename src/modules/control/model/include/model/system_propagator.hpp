/*
 * system_propagator.hpp
 *
 * Created on: Mar 21, 2018 15:15
 * Description: a convenience wrapper to propogate system model
 *            with given control input
 * Note: this propagator doesn't consider any additional constraints
 *            to system states, see comments below
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SYSTEM_PROPAGATOR_HPP
#define SYSTEM_PROPAGATOR_HPP

#include <cstdint>
#include <vector>

#include "odeint.hpp"

namespace rnav {
template <typename Model, typename Input>
class SystemPropagator {
 public:
  typename Model::state_type Propagate(typename Model::state_type init_state,
                                       Input u, double t0, double tf,
                                       double dt) {
    typename Model::state_type x = init_state;
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<typename Model::state_type>(),
        Model(u), x, t0, tf, dt);
    return x;
  }
};
}  // namespace rnav

#endif /* SYSTEM_PROPAGATOR_HPP */
