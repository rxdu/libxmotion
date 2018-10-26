/* 
 * primitive_generator.cpp
 * 
 * Created on: Oct 21, 2018 23:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/primitive_generator.hpp"

#include <numeric>

#include <eigen3/Eigen/LU>

using namespace librav;

#define ENABLE_SCALER_SELECTION

/*
 * Reference: 
 *  [1] M. McNaughton and C. Urmson and J. M. Dolan and J. W. Lee. 2011. 
 *    “Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice.” 
 *    In 2011 IEEE International Conference on Robotics and Automation, 4889–95.
 *  [2] Howard, Thomas M., and Alonzo Kelly. 2007. “Optimal Rough Terrain Trajectory Generation 
 *      for Wheeled Mobile Robots.” The International Journal of Robotics Research 26 (2): 141–66.
 *  [3] Python Robotics, https://github.com/AtsushiSakai/PythonRobotics
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
 *  (2) final condition
 *      x_p(sf) = [xf yf thetaf kappaf] = x_des
 *   From (1)   
 *      => p0 = kappa0     ... [1]
 *         p3 = kappaf     ... [2]
 *   From (2)
 *      => x_p(sf) = x_des ... [3]
 *  In total, 3 unkowns {p1, p2 ,s_f} left with 3 nonlinear system of equations 
 *      - x_p(s) = \int_{0}^{s} cos(theta_p(s)) ds 
 *      - y_p(s) = \int_{0}^{s} sin(theta_p(s)) ds
 *      - theta_p(s) = a(p)s + b(p)s^2/2 + c(p)s^3/3 + d(p)s^4/4 
 * 
 *  Formulate as root finding problem and use Newton's method to solve:
 *      g(p) = h(p) - x_des = 0
 *  (h(p) is the same with x_p(sf) but p is treated as variables instead)
 *  => p_{n+1} = p_{n} + \delta p
 *  => p_{n+1} = p_{n} - [\partial/\partial p g(p)]^{-1} g(p)
 *  where  [\partial/\partial p g(p)]\delta p = - g(p)
 */

PrimitiveGenerator::PrimitiveGenerator()
{
    Je_ << 0.02, 0.02, 0.05;

    scalers_.push_back(1.0);
    scalers_.push_back(2.0);
    scalers_.push_back(0.5);
}

bool PrimitiveGenerator::Calculate(MotionState state_s, MotionState state_f, PointKinematics::Param init_p, MotionPrimitive &mp)
{
    PointKinematics::Param p;
    p.p0 = state_s.kappa;
    p.p3 = state_f.kappa;

    StatePMatrix start;
    start << state_s.x, state_s.y, state_s.theta;
    StatePMatrix target;
    target << state_f.x, state_f.y, state_f.theta;

    ParamPMatrix p_i, delta_p_i;
    p_i << init_p.p1, init_p.p2, init_p.sf;

    StatePMatrix xp_i, xp_delta_i;
    double cost_prev = std::numeric_limits<double>::max();
    for (int32_t i = 0; i < max_iter_; ++i)
    {
        xp_i = model_.PropagateP(start, PointKinematics::Param(init_p.p0, p_i(0), init_p.p2, p_i(1), p_i(2)));
        xp_delta_i = CalcDeltaX(target, xp_i);
        double cost = xp_delta_i.norm();

        // debugging print
        // std::cout << "i = " << i << std::endl;
        // std::cout << "x_p(i): \n"
        //           << xp_i << std::endl;
        // std::cout << "delta x_p(i): \n"
        //           << xp_delta_i << std::endl;
        // std::cout << "--> cost <--: " << cost << std::endl;

        if (cost <= cost_th_)
        {
            // std::cout << "path found" << std::endl;
            mp = MotionPrimitive(state_s, state_f, PointKinematics::Param(init_p.p0, p_i(0), init_p.p2, p_i(1), p_i(2)));
            return true;
        }

        JacobianMatrix J = CalcJacobian(state_s, state_f, PointKinematics::Param(init_p.p0, p_i(0), init_p.p2, p_i(1), p_i(2)));

        Eigen::Matrix<double, 3, 3> J_inv = J.inverse();

        if (J_inv.hasNaN())
        {
            // std::cout << "failed to get inverse of J" << std::endl;
            return false;
        }

        delta_p_i = -J_inv * xp_delta_i;

#ifdef ENABLE_SCALER_SELECTION
        double scaler = SelectParamScaler(start, target, init_p, p_i, delta_p_i);
        p_i = p_i + scaler * delta_p_i;
        // std::cout << "scaler: " << scaler << std::endl;
#else
        p_i = p_i + delta_p_i;
#endif
        // std::cout << "J: \n"
        //           << J << std::endl;
        // std::cout << "delta p_i: \n"
        //           << delta_p_i << std::endl;

        // std::cout << "--------------------------------------" << std::endl;
    }

    std::cout << "failed to find a path" << std::endl;

    return false;
}

MotionPrimitive PrimitiveGenerator::Calculate(MotionState state_s, MotionState state_f, PointKinematics::Param init_p)
{
    MotionPrimitive mp;

    Calculate(state_s, state_f, init_p, mp);

    return mp;
}

StatePMatrix PrimitiveGenerator::CalcDeltaX(StatePMatrix target, StatePMatrix xi)
{
    StatePMatrix err = target - xi;
    return err;
}

// use central difference to approximate Jacobian matrix
JacobianMatrix PrimitiveGenerator::CalcJacobian(MotionState init, MotionState target, PointKinematics::Param p)
{
    StatePMatrix target_p;
    target_p << target.x, target.y, target.theta;
    StatePMatrix x_init;
    x_init << init.x, init.y, init.theta;

    // column: \partial x_p / \partial p1
    StatePMatrix state_p1 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1 + Je_(0), p.p2, p.p3, p.sf));
    StatePMatrix state_n1 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1 - Je_(0), p.p2, p.p3, p.sf));
    StatePMatrix err_p1 = CalcDeltaX(target_p, state_p1);
    StatePMatrix err_n1 = CalcDeltaX(target_p, state_n1);
    StatePMatrix err1 = (err_p1 - err_n1) / (2.0 * Je_(0));

    // column: \partial x_p / \partial p2
    StatePMatrix state_p2 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1, p.p2 + Je_(1), p.p3, p.sf));
    StatePMatrix state_n2 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1, p.p2 - Je_(1), p.p3, p.sf));
    StatePMatrix err_p2 = CalcDeltaX(target_p, state_p2);
    StatePMatrix err_n2 = CalcDeltaX(target_p, state_n2);
    StatePMatrix err2 = (err_p2 - err_n2) / (2.0 * Je_(1));

    // columm: \partial x_p / \partial sf
    StatePMatrix state_p3 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1, p.p2, p.p3, p.sf + Je_(2)));
    StatePMatrix state_n3 = model_.PropagateP(x_init, PointKinematics::Param(p.p0, p.p1, p.p2, p.p3, p.sf - Je_(2)));
    StatePMatrix err_p3 = CalcDeltaX(target_p, state_p3);
    StatePMatrix err_n3 = CalcDeltaX(target_p, state_n3);
    StatePMatrix err3 = (err_p3 - err_n3) / (2.0 * Je_(2));

    JacobianMatrix J;

    J.col(0) = err1.transpose();
    J.col(1) = err2.transpose();
    J.col(2) = err3.transpose();

    return J;
}

double PrimitiveGenerator::SelectParamScaler(StatePMatrix start, StatePMatrix target, PointKinematics::Param p, ParamPMatrix p_i, ParamPMatrix delta_pi)
{
    double selected_scaler;
    double min_cost = std::numeric_limits<double>::max();

    for (auto &scaler : scalers_)
    {
        ParamPMatrix np_i = p_i + scaler * delta_pi;

        StatePMatrix xp_i = model_.PropagateP(start, PointKinematics::Param(p.p0, np_i(0), p.p2, np_i(1), np_i(2)));
        StatePMatrix xp_delta_i = CalcDeltaX(target, xp_i);
        double cost = xp_delta_i.norm();
        if (cost < min_cost)
        {
            min_cost = cost;
            selected_scaler = scaler;
        }
    }

    return selected_scaler;
}