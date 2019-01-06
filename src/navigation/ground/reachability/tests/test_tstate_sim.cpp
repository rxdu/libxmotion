#include <iostream>

#include "reachability/details/tstate_transition_sim.hpp"
// #include "lightviz/lightviz.hpp"
// #include "navviz/navviz.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    TStateTransitionSim sim;

    sim.SetupStateSpace(0, 100, 0, 20, 20, 10);

    Eigen::VectorXd control_set;
    control_set.setZero(6);
    control_set << -1, -0.5, 0, 0.3, 0.6, 1.0;

    std::cout << "constrol set: \n"
              << control_set.transpose() << std::endl;

    sim.SetControlSet(control_set);

    stopwatch::StopWatch timer;

    Eigen::MatrixXd Psi = sim.RunSim(5);
    Eigen::MatrixXd Psi_T = sim.RunIntervalSim(5, 10);

    std::cout << "generated Psi in " << timer.toc() << " seconds." << std::endl;

    std::cout << "Psi size: " << Psi.rows() << " by " << Psi.cols() << std::endl;

    // LightViz::ShowTStateSpace(*sim.state_space_.get());
    // LightViz::ShowMatrixAsColorMap(Psi, "Psi_matrix", false);
    // LightViz::ShowMatrixAsImage(Psi);
    // LightViz::ShowMatrixAsColorMap(Psi_T, "Psi_T_matrix", false);

    return 0;
}