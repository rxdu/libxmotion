#include <iostream>

#include "reachability/details/tstate_transition_sim.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    TStateTransitionSim sim;

    sim.SetupStateSpace(0, 200, 0, 20, 50, 10);

    Eigen::VectorXd control_set;
    control_set.setZero(6);
    control_set << -0.6, -0.3, 0, 0.3, 0.6, 0.8;

    std::cout << "constrol set: \n" << control_set.transpose() << std::endl; 

    sim.SetControlSet(control_set);

    sim.RunSim(5);

    LightViz::ShowTStateSpace(*sim.state_space_.get());

    return 0;
}