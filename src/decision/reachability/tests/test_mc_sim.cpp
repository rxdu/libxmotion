#include <iostream>

#include "xmnav/reachability/monte_carlo_sim.hpp"
#include "stopwatch/stopwatch.hpp"

using namespace xmotion;

int main()
{
    MonteCarloSim sim;

    xmotion::StopWatch timer;

    timer.tic();
    for (int i = 0; i < 20; ++i)
    {
        double tf = 0.1 * i + 0.1;
        sim.RunSim(0, tf, 0.001, 10000);
    }
    std::cout << "simulation finished in " << timer.toc() << std::endl;

    return 0;
}