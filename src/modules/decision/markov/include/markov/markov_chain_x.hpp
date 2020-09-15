/* 
 * markov_chain_x.hpp
 * 
 * Created on: Oct 29, 2018 05:30
 * Description: a Markov chain model, state vector and transition
 *              matrix are represented as dynamic dense matrices,
 *              could be used for larger Markov model
 * 
 * Note: you have to resize any matrices before assigning values to them
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_CHAIN_X_HPP
#define MARKOV_CHAIN_X_HPP

#include <cstdint>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace librav
{
template <int32_t N>
class MarkovChainX
{
  public:
    using State = Eigen::MatrixXd;
    using Transition = Eigen::MatrixXd;

    MarkovChainX()
    {
        init_state_.resize(1, N);
        transition_.resize(N, N);
    }

    MarkovChainX(Transition trans) : transition_(trans)
    {
        init_state_.resize(1, N);
        transition_.resize(N, N);
    }

    MarkovChainX(State st, Transition trans) : init_state_(st), transition_(trans)
    {
        init_state_.resize(1, N);
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

    inline void Propagate() { states_.emplace_back(states_.back() * transition_); }

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

    State CalculateStateAt(int32_t k)
    {
        State s = init_state_;

        for (int32_t i = 0; i < k; ++i)
        {
            s = s * transition_;
            // normalize to avoid undersampling
            s = s / s.sum();
        }
        return s;
    }

  protected:
    State init_state_;
    Transition transition_;

    std::vector<State> states_;
};
} // namespace librav

#endif /* MARKOV_CHAIN_X_HPP */
