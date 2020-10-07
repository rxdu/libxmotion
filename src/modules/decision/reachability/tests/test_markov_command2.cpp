#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace autodrive;

int main()
{
    using CommandModel = MarkovCommand<1, 6>;

    // setup command Markov model
    std::shared_ptr<CommandModel> command = std::make_shared<CommandModel>();

    typename CommandModel::State init_state;
    init_state.resize(1, 6);
    init_state << 0, 0, 0.5, 0.5, 0, 0;

    typename CommandModel::ControlSet cmds;
    cmds.resize(6);
    // cmds << -1, -0.5, 0, 0.3, 0.6, 1.0;
    cmds << -0.6, -0.3, 0, 0.3, 0.6, 1.0;

    typename CommandModel::PriorityVector priority_vec;
    priority_vec.resize(6);
    priority_vec << 0.01, 0.04, 0.25, 0.25, 0.4, 0.05;

    double gamma = 0.2;

    command->SetupModel(init_state, cmds, priority_vec, gamma);

    std::cout << "---------------------------" << std::endl;
    for (int i = 0; i < 10; ++i)
        std::cout << "Phi(" << i << ")\n"
                  << command->CalculateStateAt(i) << "\n"
                  << std::endl;

    return 0;
}