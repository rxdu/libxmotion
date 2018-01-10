#include <iostream>

#include "traffic/traffic_sim.h"
#include "utility/stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 50

int main()
{
    std::cout << "started traffic sim" << std::endl;
    TrafficSim sim;

    sim.SetDuration(180);
    sim.SetStartTime(0);
    sim.SetStepSize(1);

    stopwatch::StopWatch timer;

    // simulation loop
    while (true)
    {
        timer.tic();

        sim.UpdateTraffic();

        timer.sleep_ms_util(LOOP_PERIOD);
    }

    return 0;
}