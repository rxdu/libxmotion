/* 
 * point_kinematics.cpp
 * 
 * Created on: Oct 24, 2018 23:09
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/details/point_kinematics.hpp"

using namespace librav;

void PointKinematics::SetParameters(const Param &p)
{
    CalculateIntermediateParams(p);
}

void PointKinematics::CalculateIntermediateParams(const Param &p)
{
    std::cout << "p: " << p << std::endl;

    a_ = p.p0;
    b_ = -(11 * p.p0 - 18 * p.p1 + 9 * p.p2 - 2 * p.p3) / (2.0 * p.sf);
    c_ = 9.0 * (2 * p.p0 - 5 * p.p1 + 4 * p.p2 - p.p3) / (2.0 * p.sf * p.sf);
    d_ = -9.0 * (p.p0 - 3 * p.p1 + 3 * p.p2 - p.p3) / (2.0 * p.sf * p.sf * p.sf);

    std::cout << "(a,b,c,d): " << a_ << " , " << b_ << " , " << c_ << " , " << d_ << std::endl;
}

// x0 = x, x1 = y
void PointKinematics::operator()(const asc::state_t &x, asc::state_t &xd, const double s)
{
    double s_squared = s * s;
    double theta_p = a_ * s + b_ * s_squared / 2.0 + c_ * s_squared * s / 3.0 + d_ * s_squared * s_squared / 4.0;

    xd[0] = std::cos(theta_p);
    xd[1] = std::sin(theta_p);
}

MotionState PointKinematics::Propagate(const MotionState &init, const Param &p, double ds)
{
    CalculateIntermediateParams(p);

    return Propagate(init, p.sf, ds);
}

StatePMatrix PointKinematics::PropagateP(const StatePMatrix &init, const Param &p, double ds)
{
    CalculateIntermediateParams(p);
    MotionState state = Propagate(MotionState(init(0), init(1)), p.sf, ds);

    StatePMatrix xpm;
    xpm << state.x, state.y, state.theta;

    return xpm;
}

std::vector<MotionState> PointKinematics::GenerateTrajectoryPoints(const MotionState &init, double sf, double step, double ds)
{
    std::vector<MotionState> states;

    for (double s = 0; s <= sf; s += step)
        states.push_back(Propagate(init, s, ds));

    return states;
}
