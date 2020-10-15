/* 
 * motion_primitive.cpp
 * 
 * Created on: Oct 25, 2018 10:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/motion_primitive.hpp"

using namespace ivnav;

MotionPrimitive::MotionPrimitive(MotionState state_s, MotionState state_f) : state_s_(state_s), state_f_(state_f)
{
}

MotionPrimitive::MotionPrimitive(MotionState state_s, MotionState state_f, PointKinematics::Param p) : state_s_(state_s), state_f_(state_f), sf_(p.sf), params_(p)
{
    model_.SetParameters(p);
}

void MotionPrimitive::SetParameters(const PointKinematics::Param &p)
{
    sf_ = p.sf;
    model_.SetParameters(p);
}

MotionState MotionPrimitive::Evaluate(double s, double ds)
{
    return model_.Propagate(state_s_, s, ds);
}