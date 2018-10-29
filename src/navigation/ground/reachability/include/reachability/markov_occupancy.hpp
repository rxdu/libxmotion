/* 
 * markov_occupancy.hpp
 * 
 * Created on: Oct 29, 2018 07:50
 * Description: markov occupancy estimation setup
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_OCCUPANCY_HPP
#define MARKOV_OCCUPANCY_HPP

#include <cstdint>

#include "reachability/details/markov_command.hpp"
#include "reachability/details/markov_motion.hpp"

namespace librav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
class MarkovOccupancy
{
  public:
    MarkovOccupancy();

  private:
    static constexpr int32_t N = 10;
    static constexpr int32_t M = 6;

    using CommandModel = MarkovCommand<N, M>;
    using MotionModel = MarkovMotion<N, M>;

    CommandModel command_;
    MotionModel motion_;

    void SetupCommandModel();
};
} // namespace librav

#endif /* MARKOV_OCCUPANCY_HPP */
