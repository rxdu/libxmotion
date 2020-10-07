#include <iostream>

#include "traffic_sim/traffic_sim_manager.hpp"
#include "traffic_sim/sim_scenarios.hpp"

using namespace autodrive;

int main()
{
    TrafficSimConfig config = SimScenario::GenerateScenarioCase2();

    /*----------------------------------------------------------------*/

    TrafficSimManager sim_manager(config);

    if (!sim_manager.ValidateSimConfig())
    {
        std::cerr << "ERROR: invalid simulation config!" << std::endl;
        return -1;
    }

    sim_manager.RunSim();

    return 0;
}