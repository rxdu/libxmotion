/* 
 * test_propagator.cpp
 * 
 * Created on: Mar 21, 2018 15:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>
#include "reachability/vehicle_dynamics.hpp"
#include "reachability/system_propagator.hpp"

using namespace librav;

int main()
{
    SystemPropagator<LongitudinalDynamics, double> propagator;

    asc::state_t state = propagator.Propagate({0.0, 8.0}, 0.1, 0, 10, 0.001);

    std::cout << "final state: " << state[0] << " , " << state[1] << std::endl;

    return 0;
}