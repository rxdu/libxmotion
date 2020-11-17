/*
 * car_model_propagator.hpp
 *
 * Created on: Oct 30, 2018 22:52
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAR_MODEL_PROPAGATOR_HPP
#define CAR_MODEL_PROPAGATOR_HPP

#include <cstdint>
#include <vector>

#include "odeint.hpp"

#include "reachability/details/car_longitudinal_model.hpp"

namespace rnav {
class CarModelPropagator {
 public:
  CarLongitudinalModel::state_type Propagate(CarLongitudinalModel::state_type init_state,
                       CarLongitudinalModel::control_type u, double t0,
                       double tf, double dt) {
    double t = t0;
    CarLongitudinalModel::state_type x = init_state;

    while (t <= tf) {
      //   integrator_(CarLongitudinalModel(u), x, t, dt);

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<
              CarLongitudinalModel::state_type>(),
          CarLongitudinalModel(u), x, t, t+dt, dt/10.0);

      // add additional constraint to s, v: s >= s0, v >=0, v < v_max
      if (x[0] < init_state[0]) x[0] = init_state[0];
      if (x[1] < 0) x[1] = 0;
      if (x[1] > CarLongitudinalModel::v_max)
        x[1] = CarLongitudinalModel::v_max;
    }

    return x;
  }
};
}  // namespace rnav

#endif /* CAR_MODEL_PROPAGATOR_HPP */
