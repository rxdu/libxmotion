/* 
 * markov_command.hpp
 * 
 * Created on: Oct 29, 2018 06:56
 * Description: Markov-chain acceleration command model
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_COMMAND_HPP
#define MARKOV_COMMAND_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "markov/markov_chain.hpp"

namespace librav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t N, int32_t M>
class MarkovCommand : public MarkovChain<M * N>
{
  public:
    using Model = MarkovChain<M * N>;
    using State = typename MarkovChain<M * N>::State;
    using Transition = typename MarkovChain<M * N>::Transition;

    using ControlSet = Eigen::Matrix<double, M, 1>;
    using PriorityVector = Eigen::Matrix<double, M, 1>;
    using InputDynamicMatrix = Eigen::Matrix<double, M, M>;

    void SetupModel(State init, ControlSet cmds, PriorityVector m, double gamma)
    {
        m_ = m;
        gamma_ = gamma;
        commands_ = cmds;

        Eigen::Matrix<double, M, M> lammda = m_.asDiagonal();
        std::cout << "lammda: \n"
                  << lammda << std::endl;

        // set initial state probability distribution
        Model::SetInitialState(init);

        // generate Psi
        Eigen::Matrix<double, M, M> Psi;
        for (int32_t alpha = 0; alpha < M; ++alpha)
            for (int32_t beta = 0; beta < M; ++beta)
                Psi(alpha, beta) = 1.0 / ((commands_(beta) - commands_(alpha)) * (commands_(beta) - commands_(alpha)) + gamma_);
        Psi.normalize();
        std::cout << "Psi: \n"
                  << Psi << std::endl;

        std::cout << "block: \n"
                  << lammda * Psi << std::endl;

        Transition trans;
        for (int32_t i = 0; i < N; ++i)
            trans.block(i * M, i * M, M, M) = lammda * Psi;
        trans.normalize();
        std::cout << "Trans: \n"
                  << trans << std::endl;

        Model::SetTransitionMatrix(trans);
    }

  private:
    ControlSet commands_;
    // note: m_ could be different for each state, here we treat them as the same
    PriorityVector m_;
    double gamma_;
};
} // namespace librav

#endif /* MARKOV_COMMAND_HPP */
