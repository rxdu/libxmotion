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

void PointKinematics::CalculateIntermediateParams(Param p)
{
    // std::cout << "p: " << p << std::endl;

    a_ = p.p0;
    b_ = -(11 * p.p0 - 18 * p.p1 + 9 * p.p2 - 2 * p.p3) / (2.0 * p.sf);
    c_ = 9.0 * (2 * p.p0 - 5 * p.p1 + 4 * p.p2 - p.p3) / (2.0 * p.sf * p.sf);
    d_ = -9.0 * (p.p0 - 3 * p.p1 + 3 * p.p2 - p.p3) / (2.0 * p.sf * p.sf * p.sf);

    // std::cout << "(a,b,c,d): " << a_ << " , " << b_ << " , " << c_ << " , " << d_ << std::endl;
}

// x0 = x, x1 = y
void PointKinematics::operator()(const asc::state_t &x, asc::state_t &xd, const double s)
{
    double s_squared = s * s;
    double theta_p = a_ * s + b_ * s_squared / 2.0 + c_ * s_squared * s / 3.0 + d_ * s_squared * s_squared;

    xd[0] = std::cos(theta_p);
    xd[1] = std::sin(theta_p);
}

MotionState PointKinematics::Propagate(MotionState init, Param p, double ds)
{
    CalculateIntermediateParams(p);

    double sf_squared = p.sf * p.sf;

    // theta_p and kappa_p could be calculated analytically
    double theta_p = a_ * p.sf + b_ * sf_squared / 2.0 + c_ * sf_squared * p.sf / 3.0 + d_ * sf_squared * sf_squared;
    double kappa_p = a_ + b_ * p.sf + c_ * sf_squared + d_ * sf_squared * p.sf;

    // calculate x_p, y_p numerically
    double s = 0;
    asc::state_t xy_p = {init.x, init.y};
    while (s < p.sf)
    {
        integrator_(*this, xy_p, s, ds);
    }

    return MotionState(xy_p[0], xy_p[1], theta_p, kappa_p);
}

StatePMatrix PointKinematics::PropagateP(StatePMatrix init, Param p, double ds)
{
    CalculateIntermediateParams(p);

    double sf_squared = p.sf * p.sf;

    // theta_p and kappa_p could be calculated analytically
    double theta_p = a_ * p.sf + b_ * sf_squared / 2.0 + c_ * sf_squared * p.sf / 3.0 + d_ * sf_squared * sf_squared;
    double kappa_p = a_ + b_ * p.sf + c_ * sf_squared + d_ * sf_squared * p.sf;

    // calculate x_p, y_p numerically
    double s = 0;
    asc::state_t xy_p = {init(0), init(1)};
    while (s < p.sf)
    {
        integrator_(*this, xy_p, s, ds);
    }

    StatePMatrix xpm;
    xpm << xy_p[0], xy_p[1], theta_p;

    return xpm;
}