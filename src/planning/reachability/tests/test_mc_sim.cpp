#include <iostream>

#include "reachability/monte_carlo_sim.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MonteCarloSim sim;

    stopwatch::StopWatch timer;

    timer.tic();
    sim.RunSim(10000);
    std::cout << "simulation finished in " << timer.toc() << std::endl;

    return 0;
}