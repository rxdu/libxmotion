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

namespace rnav
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
        sparse_init_s_ = init_state_.sparseView();
    }

    void SetInitialState(State st)
    {
        init_state_ = st;
        states_.push_back(init_state_);

        sparse_init_s_ = init_state_.sparseView();
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

    State GetStateAt(int32_t k)
    {
        assert(states_.size() > k);

        return states_[k];
    }

    State GetIntervalStateAt(int32_t km1_k)
    {
        assert(intv_states_.size() > km1_k);

        return intv_states_[km1_k];
    }

    void Propagate(int32_t k)
    {
        // stopwatch::StopWatch timer;
        states_.reserve(k + 1);

        // forward propagate for k steps
        Eigen::SparseMatrix<double> sparse_s = sparse_init_s_;
        for (int32_t i = 0; i < k; ++i)
        {
            // then calculate p(t_{k+1})
            sparse_s = sparse_trans_ * sparse_s;
            sparse_s = sparse_s / sparse_s.sum();
            states_.push_back(State(sparse_s));
        }

        // std::cout << "matrix calculation finished in " << timer.toc() << std::endl;
    }

    void PropagateWithIntervalDist(int32_t k)
    {
        // stopwatch::StopWatch timer;
        states_.reserve(k + 1);
        intv_states_.reserve(k);

        // forward propagate for k steps
        Eigen::SparseMatrix<double> sparse_s = sparse_init_s_;
        for (int32_t i = 0; i < k; ++i)
        {
            // first calculate interval distribution before update p(t_k)
            Eigen::SparseMatrix<double> intv_sparse_s = sparse_interval_trans_ * sparse_s;
            intv_sparse_s = intv_sparse_s / intv_sparse_s.sum();
            intv_states_.push_back(State(intv_sparse_s));

            // then calculate p(t_{k+1})
            sparse_s = sparse_trans_ * sparse_s;
            sparse_s = sparse_s / sparse_s.sum();
            states_.push_back(State(sparse_s));
        }

        // std::cout << "matrix calculation finished in " << timer.toc() << std::endl;
    }

    State CalculateStateAt(int32_t k)
    {
        // stopwatch::StopWatch timer;

        Eigen::SparseMatrix<double> sparse_s = sparse_init_s_;

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
    Eigen::SparseMatrix<double> sparse_init_s_;
    Eigen::SparseMatrix<double> sparse_trans_;
    Eigen::SparseMatrix<double> sparse_interval_trans_;

    std::vector<State> states_;
    std::vector<State> intv_states_;
};
} // namespace rnav

#endif /* OCCUPANCY_MARKOV_CHAIN_HPP */
