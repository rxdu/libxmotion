/* 
 * monte_carlo_sim.cpp
 * 
 * Created on: Mar 21, 2018 15:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/monte_carlo_sim.hpp"
#include "logging/logger.hpp"

using namespace librav;

// MonteCarloSim::MonteCarloSim()
// {
// }

void MonteCarloSim::RunSim(int32_t iter_num)
{
    double x = 0, y = 0;

    std::vector<double> x_random;
    std::vector<double> y_random;

    x_random.resize(iter_num);
    y_random.resize(iter_num);
    for (int32_t i = 0; i < iter_num; ++i)
    {
        sampler_.Sample(&x, &y);
        x_random[i] = x;
        y_random[i] = y;
    }

    for (int32_t i = 0; i < iter_num; ++i)
    {
        asc::state_t state = propagator_.Propagate({y_random[i], 8.0}, 0.1, 0, 2.0, 0.01);

        GlobalCsvLogger::GetLogger("monte_carlo_y", "/home/rdu").LogData(state[0], state[1]);
    }
}
