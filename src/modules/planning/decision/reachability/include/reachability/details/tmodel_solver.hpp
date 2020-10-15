/* 
 * tmodel_solver.hpp
 * 
 * Created on: Oct 30, 2018 06:06
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TMODEL_SOLVER_HPP
#define TMODEL_SOLVER_HPP

#include <cstdint>
#include <iostream>

namespace ivnav
{
struct TModelSolver
{
    // State defined the same as in TStateSpace
    struct State
    {
        State() = default;
        State(double _s, double _v) : s(_s), v(_v) {}

        double s = 0;
        double v = 0;

        friend std::ostream &operator<<(std::ostream &os, const State &state)
        {
            os << "state: (" << state.s << " , " << state.v << ")";
            return os;
        }
    };

    State Propagate(State init, double u, double t)
    {
        // if((u > 0) && )
        // how to handle model switching ?
    }

    static constexpr double v_sw = 7.3;   // switching velocity
    static constexpr double v_max = 18.0; // 18 m/s ~= 40 mph
    static constexpr double a_max = 7.0;  // 7.0 m/s^2
};
} // namespace ivnav

#endif /* TMODEL_SOLVER_HPP */
