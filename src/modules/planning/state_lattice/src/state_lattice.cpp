/* 
 * state_lattice.cpp
 * 
 * Created on: Aug 07, 2018 04:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/state_lattice.hpp"

#include <iostream>
#include <cmath>

#include "logging/loggers.hpp"

using namespace librav;

// PrimitiveGenerator StateLattice::generator(FolderPath::GetLogFolderPath() + "/lookup_table.20181026024917.data");
PrimitiveGenerator StateLattice::generator(GetDataFolderPath() + "/lattice/lookup/lookup_table.20181027112245.data");

StateLattice::StateLattice(MotionState state_s, MotionState state_f) : MotionPrimitive(state_s, state_f)
{
    dx_ = state_s_.x;
    dy_ = state_s_.y;
    dtheta_ = state_s_.theta;

    tstate_s_ = TransformToLocal(state_s_);
    tstate_f_ = TransformToLocal(state_f_);

    // std::cout << "transformed: " << std::endl;
    // std::cout << tstate_s_ << std::endl;
    // std::cout << tstate_f_ << std::endl;
    // std::cout << TransformToGlobal(tstate_s_) << std::endl;
    // std::cout << TransformToGlobal(tstate_f_) << std::endl;

    MotionPrimitive mp;
    valid_ = StateLattice::generator.Calculate(tstate_s_, tstate_f_, mp);

    if (valid_)
    {
        // std::cout << "state lattice created" << std::endl;
        SetParameters(mp.GetParameters());
    }
    else
    {
        std::cout << "failed to create state lattice" << std::endl;
    }
}

MotionState StateLattice::Evaluate(double s, double ds)
{
    MotionState local_state = model_.Propagate(tstate_s_, s, ds);
    return TransformToGlobal(local_state);
}

void StateLattice::GetPositionVector(double s, double &x, double &y)
{
    auto state = Evaluate(s);
    x = state.x;
    y = state.y;
}

void StateLattice::GetTangentVector(double s, double &x, double &y)
{
    auto state = Evaluate(s);
    x = std::cos(state.theta);
    y = std::sin(state.theta);
}

MotionState StateLattice::TransformToLocal(const MotionState &input)
{
    MotionState output;

    double tx = input.x - dx_;
    double ty = input.y - dy_;
    output.x = tx * std::cos(dtheta_) + ty * std::sin(dtheta_);
    output.y = -tx * std::sin(dtheta_) + ty * std::cos(dtheta_);
    output.theta = input.theta - dtheta_;
    output.kappa = input.kappa;

    return output;
}

MotionState StateLattice::TransformToGlobal(const MotionState &input)
{
    MotionState output;

    output.x = input.x * std::cos(dtheta_) - input.y * std::sin(dtheta_) + dx_;
    output.y = input.x * std::sin(dtheta_) + input.y * std::cos(dtheta_) + dy_;
    output.theta = input.theta + dtheta_;
    output.kappa = input.kappa;

    return output;
}

// MotionPrimitive StateLattice::TransformPrimitive(const MotionPrimitive &input, double dx, double dy, double dtheta)
// {
//     // copy all data from input first
//     MotionPrimitive output = input;

//     // clear nodes and replace with transformed ones
//     output.nodes.clear();
//     for (const auto &nd : input.nodes)
//     {
//         auto new_nd = TransformNode(nd, dx, dy, dtheta);
//         output.nodes.push_back(new_nd);
//     }

//     return output;
// }
