/* 
 * monte_carlo_sim.cpp
 * 
 * Created on: Mar 21, 2018 15:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/monte_carlo_sim.hpp"
#include "logging/loggers.hpp"

using namespace rnav;

void MonteCarloSim::RunSim(double t0, double tf, double step, int32_t iter_num)
{
    double x = 0, y = 0;
    double acc = -1, phi = 0;

    std::vector<double> x_random;
    std::vector<double> y_random;
    std::vector<double> acc_random;
    std::vector<double> ster_random;

    x_random.resize(iter_num);
    y_random.resize(iter_num);
    acc_random.resize(iter_num);
    ster_random.resize(iter_num);
    for (int32_t i = 0; i < iter_num; ++i)
    {
        init_pos_sampler_.Sample(&x, &y);
        x_random[i] = x;
        y_random[i] = y;

        while (acc < 0)
            acc_sampler_.Sample(&acc);
        steer_sampler_.Sample(&phi);

        acc_random[i] = acc;
        ster_random[i] = phi;
    }

    for (int32_t i = 0; i < iter_num; ++i)
    {
        // asc::state_t state = propagator_.Propagate({y_random[i], 8.0}, 0.1, 0, 2.0, 0.01);
        std::vector<double> state = propagator_.Propagate({x_random[i], y_random[i], 8.0, 0}, {acc_random[i], ster_random[i]}, t0, tf, step);

        GlobalCsvLogger::GetLogger("monte_carlo_kin", "/home/rdu").LogData(x_random[i], y_random[i], state[0], state[1]);
    }
}
