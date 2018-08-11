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

#include <cmath>

#include "reachability/vehicle_dynamics.hpp"
#include "reachability/system_propagator.hpp"
#include "reachability/bigaussian_sampler.hpp"
#include "reachability/gaussian_sampler.hpp"

namespace librav
{
class MonteCarloSim
{
  public:
    MonteCarloSim() = default;

    void RunSim(double t0, double tf, double step, int32_t iter_num);

  private:
    // SystemPropagator<LongitudinalDynamics, double> propagator_;
    SystemPropagator<BicycleKinematics, BicycleKinematics::control_t> propagator_;

    BiGaussianSampler init_pos_sampler_ = {0.1, 0.05, 0};
    GaussianSampler acc_sampler_ = {0.0, 2.0};
    GaussianSampler steer_sampler_ = {0.0, M_PI/36.0};
};
}

#endif /* MONTE_CARLO_SIM_HPP */
