/* 
 * occupancy_markov_chain.hpp
 * 
 * Created on: Oct 29, 2018 05:30
 * Description: a Markov chain model based on MarkovChainX in "markov" module,
 *              state vector and transition matrix are represented as dynamic 
 *              dense matrices, thus could be used for larger Markov model
 * 
 * Note: the order of calculation is different from standard Markov chain
 * 
 * Note: you have to resize any matrices before assigning values to them
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef OCCUPANCY_MARKOV_CHAIN_HPP
#define OCCUPANCY_MARKOV_CHAIN_HPP

#include <cstdint>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include "stopwatch/stopwatch.h"

namespace librav
{
template <int32_t N>
class OccupancyMarkovChain
{
  public:
    using State = Eigen::VectorXd;
    using Transition = Eigen::MatrixXd;

    OccupancyMarkovChain()
    {
        init_state_.resize(N);
        transition_.resize(N, N);
    }

    OccupancyMarkovChain(Transition trans) : transition_(trans)
    {
        init_state_.resize(N);
        transition_.resize(N, N);
    }

    OccupancyMarkovChain(State st, Transition trans) : init_state_(st), transition_(trans)
    {
        init_state_.resize(N);
        transition_.resize(N, N);

        states_.push_back(st);
    }

    void SetInitialState(State st)
    {
        init_state_ = st;
        states_.push_back(init_state_);
    }
    void SetTransitionMatrix(Transition trans) { transition_ = trans; }

    State GetInitialState() const { return init_state_; }
    Transition GetTransitionMatrix() const { return transition_; }
    int32_t GetStateNumber() const { return states_.size(); }

    inline void Propagate() { states_.emplace_back(transition_ * states_.back()); }

    void Propagate(int32_t k_f)
    {
        for (int32_t i = 0; i < k_f; ++i)
            Propagate();
    }

    State operator[](int32_t k)
    {
        if (states_.size() > k)
            return states_[k];

        return CalculateStateAt(k);
    }

    // State CalculateStateAt(int32_t k)
    // {
    //     State s = init_state_;

    //     for (int32_t i = 0; i < k; ++i)
    //     {
    //         s = transition_ * s;
    //         // normalize to avoid undersampling
    //         s = s / s.sum();
    //     }
    //     return s;
    // }

    State CalculateStateAt(int32_t k)
    {
        // stopwatch::StopWatch timer;

        Eigen::SparseMatrix<double> sparse_s = init_state_.sparseView();
        Eigen::SparseMatrix<double> sparse_trans = transition_.sparseView();

        for (int32_t i = 0; i < k; ++i)
        {
            sparse_s = sparse_trans * sparse_s;
            // normalize to avoid undersampling
            sparse_s = sparse_s / sparse_s.sum();
        }
        // std::cout << "matrix calculation finished in " << timer.toc() << std::endl;
        return State(sparse_s);
    }

  protected:
    State init_state_;
    Transition transition_;

    std::vector<State> states_;
};
} // namespace librav

#endif /* OCCUPANCY_MARKOV_CHAIN_HPP */
