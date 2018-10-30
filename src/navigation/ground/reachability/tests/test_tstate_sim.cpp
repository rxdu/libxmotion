#include <iostream>

#include "reachability/details/tstate_transition_sim.hpp"

using namespace librav;

int main()
{
    TStateTransitionSim sim;

    sim.SetupStateSpace(0, 50, 0, 20, 10, 10);

    Eigen::VectorXd control_set;
    control_set.setZero(6);
    control_set << -1, -0.5, 0, 0.3, 0.6, 1.0;

    std::cout << "constrol set: \n" << control_set.transpose() << std::endl; 

    sim.SetControlSet(control_set);

    sim.RunSim(0.5);

    return 0;
}