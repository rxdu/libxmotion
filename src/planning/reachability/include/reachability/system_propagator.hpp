/* 
 * system_propagator.hpp
 * 
 * Created on: Mar 21, 2018 15:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SYSTEM_PROPAGATOR_HPP
#define SYSTEM_PROPAGATOR_HPP

#include <cstdint>
#include <vector>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

namespace librav
{
template <typename SystemDynamics, typename ControlInput>
class SystemPropagator
{
public:
  asc::state_t Propagate(asc::state_t init_state, ControlInput u, double t0, double tf, double dt)
  {
    double t = t0;
    asc::state_t x = init_state;

    while (t < tf)
    {
      integrator_(SystemDynamics(u), x, t, dt);
    }

    return x;
  }

private:
  asc::RK4 integrator_;
};
}

#endif /* SYSTEM_PROPAGATOR_HPP */
