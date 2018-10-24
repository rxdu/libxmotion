/* 
 * primitive_generator.cpp
 * 
 * Created on: Oct 21, 2018 23:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/primitive_generator.hpp"

using namespace librav;

/*
 * Reference: 
 *  [1] M. McNaughton and C. Urmson and J. M. Dolan and J. W. Lee. 2011. 
 *    “Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice.” 
 *    In 2011 IEEE International Conference on Robotics and Automation, 4889–95.
 * 
 *  System model
 *  System state: x = {x, y, theta, kappa}
 *  Control input: u = {u, v}
 *  - x_dot = v(t) * cos(theta(t))
 *  - y_dot = v(t) * sin(theta(t))
 *  - theta_dot = kappa(t)
 *  - kappa_dot = u(t)
 *  Given x_0, x_f, find u(t) and v(t).
 * 
 *  To solve the BVP, transcribe the above problem to be
 *  - kappa(s) = a + bs + cs^2 + ds^3
 *  - theta(s) = as + bs^2/2 + cs^3/3 + ds^4/4
 *  - x(s) = \int_{0}^{s} cos(theta(s))
 *  - y(s) = \int_{0}^{s} sin(theta(s))
 *  Given x_0, x_f, find kappa(s), s_f
 * 
 *  Re-parameterize kappa(s) with p = [p0 p1 p2 p3 sf]
 *      for better numerical stability
 *   kappa(s) = a + bs + cs^2 + ds^3    =>
 *   kappa(s) = a(p) + b(p)s + c(p)s^2 + d(p)s^3 with
 *      - k(0) = p0
 *      - k(sf/3) = p1
 *      - k(2sf/3) = p2
 *      - k(sf) = p3
 *  Then the resulting polynomial coefficients become
 *  - a(p) = p0
 *  - b(p) = -(11p0 - 18p1 +9p2 - 2p3)/(2sf)
 *  - c(p) = 9(2p0 - 5p2 + 4p2 -p3)/(2sf^2)
 *  - d(p) = -9(p0 - 3p1 + 3p2 - p3)/(2sf^3)
 *  Since p0 = kappa(0), p3 = kappa(sf) are given, we only need 
 *      to determine p = [p1 p2 sf]
 * 
 *  Now the problem to be solved becomes:
 *  - kappa_p(s) = a(p) + b(p)s + c(p)s^2 + d(p)s^3
 *  - theta_p(s) = a(p)s + b(p)s^2/2 + c(p)s^3/3 + d(p)s^4/4 
 *  - x_p(s) = \int_{0}^{s} cos(theta_p(s)) ds 
 *  - y_p(s) = \int_{0}^{s} sin(theta_p(s)) ds
 *  Given [x_p(0) y_p(0) theta_p(0) kappa_p(0)] = [x0 y0 theta0 kappa0],
 *    [x_p(sf) y_p(sf) theta_p(sf) kappa_p(sf)] = [xf yf thetaf kappaf],  
 *  Find {p1, p2 ,s_f}
 * 
 *  For the constraint satisfaction case, we have 2 BCs:
 *  (1) initial condition
 *      x_p(0) = [x0 y0 theta0 kappa0] 
 *      => p0 = kappa0     ... [1]
 *         p3 = kappaf     ... [2]
 *  (2) final condition
 *      x_p(sf) = [xf yf thetaf kappaf] = x_des
 *      => x_p(sf) = x_des ... [3]
 *  In total, 3 unkowns left with nonlinear system of equations [3] 
 *      the 3 unkowns {p1, p2 ,s_f}.
 * 
 *  Formulate as root finding problem and use Euler's method to solve:
 *      g(p) = h(p) - x_des = 0
 *  (h(p) is the same with x_p(sf) but p is treated as variables instead)
 *  => p_{n+1} = p_{n} + \delta p
 *  => p_{n+1} = p_{n} - [\partial/\partial p g(p)]^{-1} g(p)
 *  where  [\partial/\partial p g(p)]\delta p = - g(p)
 */

MotionPrimitive PrimitiveGenerator::Calculate(State ss, State sf)
{
}
