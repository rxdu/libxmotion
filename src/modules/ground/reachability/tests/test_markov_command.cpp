#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    using CommandModel = MarkovCommand<1, 3>;

    // setup command Markov model
    std::shared_ptr<CommandModel> command = std::make_shared<CommandModel>();

    typename CommandModel::State init_state;
    init_state.resize(1, 3);
    init_state << 0, 0.8, 0.2;

    typename CommandModel::ControlSet cmds;
    cmds.resize(3);
    cmds << 1, 2, 3;

    typename CommandModel::PriorityVector priority_vec;
    priority_vec.resize(3);
    priority_vec << 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0;

    double gamma = 0.2;

    command->SetupModel(init_state, cmds, priority_vec, gamma);

    std::cout << "---------------------------" << std::endl;
    for (int i = 0; i < 10; ++i)
        std::cout << "Phi(" << i << ")\n"
                  << command->CalculateStateAt(i) << std::endl;

    return 0;
}