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
#include <memory>

#include "reachability/details/tstate_space.hpp"
#include "reachability/details/markov_command.hpp"
#include "reachability/details/markov_motion.hpp"

namespace librav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t SSize, int32_t VSize>
class MarkovOccupancy
{
  public:
    MarkovOccupancy(double smin, double smax, double vmin, double vmax) : state_space_(new TStateSpace(smin, smax, vmin, vmax)) {}

    void SetupMarkovModel()
    {
        // discretize state space
        state_space_->DiscretizeSpaceBySize(SSize, VSize);

        SetupCommandModel();
        SetupMotionModel();
    }

  private:
    // Tangential state space
    std::unique_ptr<TStateSpace> state_space_;

    // Markov model for occupancy estimation
    static constexpr int32_t N = SSize * VSize;
    static constexpr int32_t M = 6;

    using CommandModel = MarkovCommand<N, M>;
    using MotionModel = MarkovMotion<N, M>;

    CommandModel command_;
    MotionModel motion_;

    void SetupCommandModel()
    {
        typename CommandModel::State init_state;
        init_state << 0, 0, 0.5, 0.5, 0, 0;

        typename CommandModel::ControlSet cmds;
        cmds << -1, -0.5, 0, 0.3, 0.6, 1.0;

        typename CommandModel::PriorityVector priority_vec;
        priority_vec << 0.01, 0.04, 0.25, 0.25, 0.4, 0.05;

        double gamma = 0.2;

        command_.SetupModel(init_state, cmds, priority_vec, gamma);
    }

    void SetupMotionModel()
    {

    }
};
} // namespace librav

#endif /* MARKOV_OCCUPANCY_HPP */
