/* 
 * markov_motion.hpp
 * 
 * Created on: Oct 29, 2018 08:39
 * Description: Markov-chain longitudinal motion model
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_MOTION_HPP
#define MARKOV_MOTION_HPP

#include <cstdint>

#include "markov/markov_chain_x.hpp"

namespace librav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t N, int32_t M>
class MarkovMotion : public MarkovChainX<M * N>
{
  public:
    using Model = MarkovChainX<M * N>;
    using State = typename MarkovChainX<M * N>::State;
    using Transition = typename MarkovChainX<M * N>::Transition;

  public:
    void SetupModel(State init)
    {
        
    }
};
} // namespace librav

#endif /* MARKOV_MOTION_HPP */
