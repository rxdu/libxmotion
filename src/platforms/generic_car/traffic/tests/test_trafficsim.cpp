#include <iostream>

#include "traffic/traffic_sim.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
    std::cout << "started traffic sim" << std::endl;
    TrafficSim sim;

    sim.SetDuration(180);
    sim.SetStartTime(0);
    sim.SetStepSize(1);

    // simulation loop
    while (true)
    {
        sim.UpdateTraffic(LOOP_PERIOD);
    }

    return 0;
}