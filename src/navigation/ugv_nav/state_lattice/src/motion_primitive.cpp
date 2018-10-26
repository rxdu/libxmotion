/* 
 * motion_primitive.cpp
 * 
 * Created on: Oct 25, 2018 10:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/motion_primitive.hpp"

using namespace librav;

MotionPrimitive::MotionPrimitive(MotionState ss, MotionState sf, PointKinematics::Param p) : state_s_(ss), state_f_(sf), sf_(p.sf), params_(p)
{
    model_.SetParameters(p);
}

MotionState MotionPrimitive::Evaluate(double s, double ds)
{
    return model_.Propagate(state_s_, s, ds);
}