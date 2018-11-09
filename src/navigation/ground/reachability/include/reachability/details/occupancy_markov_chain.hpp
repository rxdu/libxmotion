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

        sparse_s_ = init_state_.sparseView();
    }

    void SetTransitionMatrix(Transition trans)
    {
        transition_ = trans;
        sparse_trans_ = transition_.sparseView();
    }

    void SetIntervalTransitionMatrix(Transition trans)
    {
        interval_transition_ = trans;
        sparse_interval_trans_ = interval_transition_.sparseView();
    }

    State GetInitialState() const { return init_state_; }
    Transition GetTransitionMatrix() const { return transition_; }
    int32_t GetStateNumber() const { return states_.size(); }

    State operator[](int32_t k)
    {
        if (states_.size() > k)
            return states_[k];

        return CalculateStateAt(k);
    }

    State CalculateStateAt(int32_t k)
    {
        // stopwatch::StopWatch timer;

        Eigen::SparseMatrix<double> sparse_s = sparse_s_;

        for (int32_t i = 0; i < k; ++i)
        {
            sparse_s = sparse_trans_ * sparse_s;
            // normalize to avoid undersampling
            sparse_s = sparse_s / sparse_s.sum();
        }
        // std::cout << "matrix calculation finished in " << timer.toc() << std::endl;
        return State(sparse_s);
    }

    State CalculateIntervalState(State p_tk)
    {
        Eigen::SparseMatrix<double> sparse_s = p_tk.sparseView();

        sparse_s = sparse_interval_trans_ * sparse_s;
        sparse_s = sparse_s / sparse_s.sum();

        return State(sparse_s);
    }

  protected:
    State init_state_;
    Transition transition_;
    Transition interval_transition_;

    // sparse version
    Eigen::SparseMatrix<double> sparse_s_;
    Eigen::SparseMatrix<double> sparse_trans_;
    Eigen::SparseMatrix<double> sparse_interval_trans_;

    std::vector<State> states_;
};
} // namespace librav

#endif /* OCCUPANCY_MARKOV_CHAIN_HPP */
