#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    const int32_t SSize = 10;
    const int32_t VSize = 5;
    const int32_t N = SSize * VSize;
    const int32_t M = 6;

    using CommandModel = MarkovCommand<N, M>;
    using MotionModel = MarkovMotion<N, M>;

    // setup command Markov model
    std::shared_ptr<CommandModel> command = std::make_shared<CommandModel>();

    typename CommandModel::State init_state;
    init_state.resize(M);
    init_state << 0, 0, 0.5, 0.5, 0, 0;

    typename CommandModel::ControlSet cmds;
    cmds.resize(M);
    // cmds << -1, -0.5, 0, 0.3, 0.6, 1.0;
    cmds << -0.6, -0.3, 0, 0.3, 0.6, 1.0;

    typename CommandModel::PriorityVector priority_vec;
    priority_vec.resize(M);
    priority_vec << 0.01, 0.04, 0.25, 0.25, 0.4, 0.05;

    double gamma = 0.2;

    command->SetupModel(init_state, cmds, priority_vec, gamma);

    std::cout << "---------------------------" << std::endl;
    // for (int i = 0; i < 10; ++i)
    //     std::cout << "Phi(" << i << ")\n"
    //               << command->CalculateStateAt(i) << "\n"
    //               << std::endl;

    //*******************************************************************

    std::shared_ptr<TStateSpace> state_space = std::make_shared<TStateSpace>(0, 50, 0, 20);
    state_space->DiscretizeSpaceBySize(SSize, VSize);

    TStateTransitionSim sim;
    sim.SetupStateSpace(state_space);
    sim.SetControlSet(cmds);
    Eigen::MatrixXd Psi = sim.RunSim(0.5);

    //*******************************************************************

    MotionModel motion;
    motion.SetupModel(state_space, Psi, command, 42, 3 * 3, 10, 1 * 1);

    std::cout << "s size: " << state_space->GetSSize() << std::endl;
    std::cout << "---------------------------" << std::endl;

    // for (int k = 0; k < 5; k++)
    int step = 2;
    for (int i = 0; i < 3; ++i)
    {
        int k = i * step;
        typename MotionModel::State statef = motion.CalculateStateAt(k);

        Eigen::VectorXd pos_prob_vec;
        pos_prob_vec.setZero(state_space->GetSSize());
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                pos_prob_vec(i / state_space->GetVSize()) += statef(i * M + j);

        // Eigen::VectorXd state_prob_vec;
        // state_prob_vec.setZero(N);
        // std::cout << "state size: " << statef.size() << " , state vec size: " << state_prob_vec.size() << std::endl;
        // for (int i = 0; i < N; ++i)
        //     for (int j = 0; j < M; ++j)
        //         state_prob_vec(i) += statef(i * M + j);
        // Eigen::VectorXd pos_prob_vec;
        // pos_prob_vec.setZero(state_space->GetSSize());
        // for (int i = 0; i < N; ++i)
        //     pos_prob_vec(i/state_space->GetVSize()) += state_prob_vec(i);

        std::cout << "position distribution k = " << k << " \n"
                  << pos_prob_vec << std::endl;
        std::cout << " - sum: " << pos_prob_vec.sum() << "\n"
                  << std::endl;
    }

    return 0;
}