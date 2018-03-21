/* 
 * monte_carlo_sim.hpp
 * 
 * Created on: Mar 21, 2018 15:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MONTE_CARLO_SIM_HPP
#define MONTE_CARLO_SIM_HPP

#include "reachability/vehicle_dynamics.hpp"
#include "reachability/system_propagator.hpp"
#include "reachability/bigaussian_sampler.hpp"

namespace librav
{
class MonteCarloSim
{
  public:
    MonteCarloSim() = default;

    void RunSim(int32_t iter_num);

  private:
    SystemPropagator<LongitudinalDynamics, double> propagator_;
    BiGaussianSampler sampler_ = {0.1, 0.5, 0};
};
}

#endif /* MONTE_CARLO_SIM_HPP */
