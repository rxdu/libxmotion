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

#include "xmnav/model/bicycle_model.hpp"
#include "xmnav/model/system_propagator.hpp"

#include "xmnav/random/bigaussian_sampler.hpp"
#include "xmnav/random/gaussian_sampler.hpp"

namespace xmotion
{
class MonteCarloSim
{
  public:
    void RunSim(double t0, double tf, double step, int32_t iter_num);

  private:
    // SystemPropagator<LongitudinalDynamics, double> propagator_;
    SystemPropagator<BicycleKinematics> propagator_;

    BiGaussianSampler init_pos_sampler_ = {0.1, 0.05, 0};
    GaussianSampler acc_sampler_ = {0.0, 2.0};
    GaussianSampler steer_sampler_ = {0.0, M_PI/36.0};
};
}

#endif /* MONTE_CARLO_SIM_HPP */
