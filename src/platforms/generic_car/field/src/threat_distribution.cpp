/* 
 * threat_distribution.cpp
 * 
 * Created on: Jan 23, 2018 13:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "field/threat_distribution.hpp"

#include <cmath>
#include <iostream>

using namespace librav;

/****************************************************************/
/*                  GaussianPositionThreat                      */
/****************************************************************/
GaussianPositionThreat::GaussianPositionThreat(double pos_x, double pos_y, double sigma1, double sigma2) : pos_x_(pos_x),
                                                                                                           pos_y_(pos_y),
                                                                                                           sigma_1_(sigma1),
                                                                                                           sigma_2_(sigma2)
{
    // update coeff1_ and coeff2_
    SetParameters(pos_x, pos_y, sigma1, sigma2);
}

void GaussianPositionThreat::SetParameters(double pos_x, double pos_y, double sigma1, double sigma2)
{
    pos_x_ = pos_x;
    pos_y_ = pos_y;
    sigma_1_ = sigma1;
    sigma_2_ = sigma2;

    coeff1_ = (2 * M_PI * sigma_1_ * sigma_2_);
    coeff2_ = -(2 * sigma_1_ * sigma_1_);
    coeff3_ = -(2 * sigma_2_ * sigma_2_);
}

double GaussianPositionThreat::operator()(double x, double y)
{
    double x_err = x - pos_x_;
    double y_err = y - pos_y_;

    double val = std::exp(x_err * x_err / coeff2_ + y_err * y_err / coeff3_) / coeff1_;

    return val;
}

/****************************************************************/
/*              GaussianPositionVelocityThreat                  */
/****************************************************************/
GaussianPositionVelocityThreat::GaussianPositionVelocityThreat(
    double pos_x, double pos_y, double vel_x, double vel_y) : pos_x_(pos_x),
                                                              pos_y_(pos_y),
                                                              vel_x_(vel_x),
                                                              vel_y_(vel_y)
{
    // update coeff1_ and coeff2_
    SetParameters(pos_x, pos_y, vel_x, vel_y);
}

void GaussianPositionVelocityThreat::SetParameters(double pos_x, double pos_y, double vel_x, double vel_y)
{
    pos_x_ = pos_x;
    pos_y_ = pos_y;
    vel_x_ = vel_x;
    vel_y_ = vel_y;
    rho_ = 0;

    double one_minus_rho_s = std::sqrt(1 - rho_ * rho_);

    coeff1_ = (2 * M_PI * sigma_1_ * sigma_2_) * one_minus_rho_s;
    coeff2_ = -(2 * sigma_1_ * sigma_1_ * one_minus_rho_s);
    coeff3_ = -(2 * sigma_2_ * sigma_2_ * one_minus_rho_s);
    coeff4_ = rho_ / (sigma_1_ * sigma_2_ * one_minus_rho_s);
}

double GaussianPositionVelocityThreat::operator()(double x, double y)
{
    // transform position to align with velocity vector
    // double theta = std::atan2(vel_y_, vel_x_);
    // std::cout << "theta: " << theta << std::endl;
    // double x_hat = x * std::cos(theta) + y * std::sin(theta);
    // double y_hat = - x * std::sin(theta) + y * std::cos(theta);

    double x_err = x - pos_x_;
    double y_err = y - pos_y_;

    double val = std::exp(x_err * x_err / coeff2_ + x_err * y_err * coeff4_ + y_err * y_err / coeff3_) / coeff1_;

    return val;
}