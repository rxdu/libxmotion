/* 
 * threat_distribution.cpp
 * 
 * Created on: Jan 23, 2018 13:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/threat_distribution.hpp"

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
    double pos_x, double pos_y, double vel_x, double vel_y, double sig1, double sig2) : pos_x_(pos_x),
                                                                                        pos_y_(pos_y),
                                                                                        vel_x_(vel_x),
                                                                                        vel_y_(vel_y),
                                                                                        sigma_1_(sig1),
                                                                                        sigma_2_(sig2)
{
    // update coeff1_ and coeff2_
    SetParameters(pos_x, pos_y, vel_x, vel_y, sig1, sig2);
}

void GaussianPositionVelocityThreat::SetParameters(double pos_x, double pos_y, double vel_x, double vel_y, double sigp, double sigv)
{
    pos_x_ = pos_x;
    pos_y_ = pos_y;
    vel_x_ = vel_x;
    vel_y_ = vel_y;
    sigma_1_ = sigp;
    sigma_2_ = sigv;
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
    double theta = std::atan2(vel_y_, vel_x_);
    double x_hat = (x - pos_x_) * std::cos(theta) + (y - pos_y_) * std::sin(theta);
    double y_hat = -(x - pos_x_) * std::sin(theta) + (y - pos_y_) * std::cos(theta);

    double x_err = x_hat;
    double y_err = y_hat;

    double val = std::exp(x_err * x_err / coeff2_ + x_err * y_err * coeff4_ + y_err * y_err / coeff3_) / coeff1_;

    return val;
}

/****************************************************************/
/*                    BiasedGaussianThreat                      */
/****************************************************************/
BiasedGaussianThreat::BiasedGaussianThreat(
    double pos_x, double pos_y, double vel_x, double vel_y, double sig1, double sig2) : pos_x_(pos_x),
                                                                                               pos_y_(pos_y),
                                                                                               vel_x_(vel_x),
                                                                                               vel_y_(vel_y),
                                                                                               sigma_1_(sig1),
                                                                                               sigma_2_(sig2)
{
    // update coeff1_ and coeff2_
    SetParameters(pos_x, pos_y, vel_x, vel_y, sig1, sig2);
}

void BiasedGaussianThreat::SetParameters(double pos_x, double pos_y, double vel_x, double vel_y, double sigpx, double sigpy)
{
    pos_x_ = pos_x;
    pos_y_ = pos_y;
    vel_x_ = vel_x;
    vel_y_ = vel_y;
    rho_ = 0;
    sigma_1_ = sigpx;
    sigma_2_ = sigpy;

    double one_minus_rho_s = std::sqrt(1 - rho_ * rho_);

    coeff1_ = (2 * M_PI * sigma_1_ * sigma_2_) * one_minus_rho_s;
    coeff2_ = sigma_1_ * sigma_1_;
    coeff3_ = sigma_2_ * sigma_2_;
}

double BiasedGaussianThreat::operator()(double x, double y)
{
    double x_err = x - pos_x_;
    double y_err = y - pos_y_;

    double alpha = 0.5;
    double val = std::exp(-x_err * x_err / coeff2_ - y_err * y_err / coeff3_) /
                 (1 + std::exp(-alpha * (vel_x_ * x_err + vel_y_ * y_err)));

    return val;
}