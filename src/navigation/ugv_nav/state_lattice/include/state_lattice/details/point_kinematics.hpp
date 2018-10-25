/* 
 * point_kinematics.hpp
 * 
 * Created on: Oct 24, 2018 23:05
 * Description: (reduced) point kinematics model, used for lattice generation
 * 
 * Reference: 
 *  [1] M. McNaughton and C. Urmson and J. M. Dolan and J. W. Lee. 2011. 
 *    “Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice.” 
 *    In 2011 IEEE International Conference on Robotics and Automation, 4889–95.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POINT_KINEMATICS_HPP
#define POINT_KINEMATICS_HPP

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

#include "state_lattice/details/motion_state.hpp"

namespace librav
{
class PointKinematics
{
  public:
    struct Param
    {
        Param(double _p0 = 0, double _p1 = 0, double _p2 = 0, double _p3 = 0, double _sf = 0) : p0(_p0), p1(_p1), p2(_p2), p3(_p3), sf(_sf) {}

        double p0;
        double p1;
        double p2;
        double p3;
        double sf;

        friend std::ostream &operator<<(std::ostream &os, const Param &p)
        {
            os << p.p0 << " , " << p.p1 << " , " << p.p2 << " , " << p.p3 << " , " << p.sf;
            return os;
        }
    };

    /////////////////////////////////////////////////////

  public:
    PointKinematics() = default;

    MotionState Propagate(MotionState init, Param p, double ds = 0.1);
    StatePMatrix PropagateP(StatePMatrix init, Param p, double ds = 0.1);

    // shall not be called by user, used for propagation by RK4 class
    void operator()(const asc::state_t &x, asc::state_t &xd, const double s);

  private:
    double a_;
    double b_;
    double c_;
    double d_;

    asc::RK4 integrator_;

    void CalculateIntermediateParams(Param p);
};
} // namespace librav

#endif /* POINT_KINEMATICS_HPP */
