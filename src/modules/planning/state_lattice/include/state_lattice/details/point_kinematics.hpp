/*
 * point_kinematics.hpp
 *
 * Created on: Oct 24, 2018 23:05
 * Description: (reduced) point kinematics model, used for lattice generation
 *
 * Reference:
 *  [1] M. McNaughton and C. Urmson and J. M. Dolan and J. W. Lee. 2011.
 *    “Motion Planning for Autonomous Driving with a Conformal Spatiotemporal
 * Lattice.” In 2011 IEEE International Conference on Robotics and Automation,
 * 4889–95.
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POINT_KINEMATICS_HPP
#define POINT_KINEMATICS_HPP

#include <iostream>

#include <eigen3/Eigen/Dense>

#include <boost/numeric/odeint.hpp>

#include "state_lattice/details/motion_state.hpp"

namespace rnav {
class PointKinematics {
 public:
  struct Param {
    Param(double _p0 = 0, double _p1 = 0, double _p2 = 0, double _p3 = 0,
          double _sf = 0)
        : p0(_p0), p1(_p1), p2(_p2), p3(_p3), sf(_sf) {}

    double p0;
    double p1;
    double p2;
    double p3;
    double sf;

    friend std::ostream &operator<<(std::ostream &os, const Param &p) {
      os << "(p0,p1,p2,p3,sf): " << p.p0 << " , " << p.p1 << " , " << p.p2
         << " , " << p.p3 << " , " << p.sf;
      return os;
    }
  };

  using state_type = std::vector<double>;

  /////////////////////////////////////////////////////

 public:
  PointKinematics() = default;
  PointKinematics(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d){};

  // calculate intermediate parameters
  void SetParameters(const Param &p);

  // propagate system model
  MotionState Propagate(const MotionState &init, const Param &p,
                        double ds = 0.1);
  StatePMatrix PropagateP(const StatePMatrix &init, const Param &p,
                          double ds = 0.1);

  // The following functions could be called externally ONLY when a_,b_,c_,d_
  // have been set properly by
  //  (1) construct the model with: PointKinematics(double a, double b, double
  //  c, double d) (2) default construct and then use SetParameters() to set
  //  parameters
  inline MotionState Propagate(const MotionState &init, double sf, double ds) {
    {
      double sf_squared = sf * sf;

      // theta_p and kappa_p could be calculated analytically
      double theta_p = a_ * sf + b_ * sf_squared / 2.0 +
                       c_ * sf_squared * sf / 3.0 +
                       d_ * sf_squared * sf_squared / 4.0;
      double kappa_p = a_ + b_ * sf + c_ * sf_squared + d_ * sf_squared * sf;

      // calculate x_p, y_p numerically
      double s = 0;
      state_type x = {init.x, init.y};
      while (s <= sf) {
        // integrator_(*this, x, s, ds);
        boost::numeric::odeint::integrate_const(
            boost::numeric::odeint::runge_kutta4<state_type>(), *this, x, s,
            s + ds, ds / 10.0);
      }

      return MotionState(x[0], x[1], theta_p, kappa_p);
    }
  }

  std::vector<MotionState> GenerateTrajectoryPoints(const MotionState &init,
                                                    double sf, double step,
                                                    double ds = 0.1);

  // Shall not be called by user, used for propagation by RK4 integrator class
  void operator()(const state_type &x, state_type &xd, const double s);

 private:
  double a_;
  double b_;
  double c_;
  double d_;

  inline void CalculateIntermediateParams(const Param &p);
};
}  // namespace rnav

#endif /* POINT_KINEMATICS_HPP */
