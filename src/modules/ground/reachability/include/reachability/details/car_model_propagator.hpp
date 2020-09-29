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

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

#include "reachability/details/car_longitudinal_model.hpp"

namespace librav
{
class CarModelPropagator
{
public:
  asc::state_t Propagate(asc::state_t init_state, CarLongitudinalModel::control_t u, double t0, double tf, double dt)
  {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf)
    {
      integrator_(CarLongitudinalModel(u), x, t, dt);

      // add additional constraint to s, v: s >= s0, v >=0, v < v_max
      if(x[0] < init_state[0])
        x[0] = init_state[0];
      if (x[1] < 0)
        x[1] = 0;
      if (x[1] > CarLongitudinalModel::v_max)
        x[1] = CarLongitudinalModel::v_max;
    }

    return x;
  }

private:
  asc::RK4 integrator_;
};
} // namespace librav

#endif /* CAR_MODEL_PROPAGATOR_HPP */
