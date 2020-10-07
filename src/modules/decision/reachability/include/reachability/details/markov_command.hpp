/* 
 * markov_command.hpp
 * 
 * Created on: Oct 29, 2018 06:56
 * Description: Markov-chain acceleration command model
 * 
 * Reference:
 *  [1] Althoff, M., and A. Mergel. 2011. “Comparison of Markov Chain Abstraction 
 *      and Monte Carlo Simulation for the Safety Assessment of Autonomous Cars.” 
 *      IEEE Transactions on Intelligent Transportation Systems 12 (4): 1237–47.
 *  [2] Althoff, M., O. Stursberg, and M. Buss. 2009. “Model-Based Probabilistic 
 *      Collision Detection in Autonomous Driving.” IEEE Transactions on Intelligent 
 *      Transportation Systems 10 (2): 299–310.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_COMMAND_HPP
#define MARKOV_COMMAND_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "markov/markov_chain_x.hpp"

// #define PRINT_MATRIX

namespace autodrive
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t N, int32_t M>
class MarkovCommand : public MarkovChainX<M * N>
{
  public:
    using Model = MarkovChainX<M * N>;
    using State = typename MarkovChainX<M * N>::State;
    using Transition = typename MarkovChainX<M * N>::Transition;

    using ControlSet = Eigen::VectorXd;
    using PriorityVector = Eigen::VectorXd;
    using InputDynamicMatrix = Eigen::MatrixXd;

    MarkovCommand() : MarkovChainX<M * N>()
    {
        commands_.resize(M);
        m_.resize(M);
    }

    void SetupModel(State init, ControlSet cmds, PriorityVector m, double gamma)
    {
        m_ = m;
        gamma_ = gamma;
        commands_ = cmds;

        Eigen::Matrix<double, M, M> lammda = m_.asDiagonal();

        // set initial state probability distribution
        Model::SetInitialState(init);

        // generate Phi
        Eigen::Matrix<double, M, M> Phi;
        for (int32_t alpha = 0; alpha < M; ++alpha)
            for (int32_t beta = 0; beta < M; ++beta)
                Phi(alpha, beta) = 1.0 / ((commands_(beta) - commands_(alpha)) * (commands_(beta) - commands_(alpha)) + gamma_);
        Phi = Phi / Phi.sum();

        // normalize by column, shall NOT normalize by whole matrix below
        Eigen::MatrixXd phi_block = lammda * Phi;
        for (int i = 0; i < phi_block.cols(); ++i)
            phi_block.col(i) = phi_block.col(i) / phi_block.col(i).sum();

#ifdef PRINT_MATRIX
        std::cout << "lammda: \n"
                  << lammda << std::endl;
        std::cout << "Phi: \n"
                  << Phi << std::endl;
        std::cout << "phi block: \n"
                  << phi_block << std::endl;
#endif

        Transition state_trans;
        state_trans.resize(M * N, M * N);
        for (int32_t i = 0; i < N; ++i)
            state_trans.block(i * M, i * M, M, M) = phi_block;
 
        // std::cout << "Trans: \n"
        //           << state_trans << std::endl;

        Model::SetTransitionMatrix(state_trans);
    }

  private:
    ControlSet commands_;
    // note: m_ could be different for each state, here we treat them as the same
    PriorityVector m_;
    double gamma_;
};
} // namespace autodrive

#endif /* MARKOV_COMMAND_HPP */
