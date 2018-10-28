/* 
 * motion_state.hpp
 * 
 * Created on: Oct 25, 2018 00:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_STATE_HPP
#define MOTION_STATE_HPP

#include <iostream>

#include <eigen3/Eigen/Dense>

namespace librav
{
using JacobianMatrix = Eigen::Matrix<double, 3, 3>;
using StatePMatrix = Eigen::Vector3d;
using ParamPMatrix = Eigen::Matrix<double, 3, 1>;

struct StateP
{
    StateP() : x(0), y(0), theta(0) {}
    StateP(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}

    double x;
    double y;
    double theta;
};

struct MotionState
{
    MotionState() : x(0), y(0), theta(0), kappa(0) {}
    MotionState(double _x, double _y, double _theta = 0, double _kappa = 0) : x(_x), y(_y), theta(_theta), kappa(_kappa) {}

    double x;
    double y;
    double theta;
    double kappa;

    friend std::ostream &operator<<(std::ostream &os, const MotionState &state)
    {
        os << "(x,y,theta,kappa): " << state.x << " , " << state.y << " , " << state.theta << " , " << state.kappa;
        return os;
    }
};
} // namespace librav

#endif /* MOTION_STATE_HPP */
