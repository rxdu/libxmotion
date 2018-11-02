/* 
 * gaussian_distribution.cpp
 * 
 * Created on: Jan 23, 2018 13:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "random/gaussian_distribution.hpp"

#include <cmath>
#include <iostream>

using namespace librav;

GaussianDistribution::GaussianDistribution(double mean, double sd)
{
}

void GaussianDistribution::SetParameters(double m, double sd)
{
}

////////////////////////////////////////////////////////////////////////

BiGaussianDistribution::BiGaussianDistribution(double mx, double my, CovarMatrix covar)
{
    SetParameters(mx, my, covar);
}

void BiGaussianDistribution::SetParameters(double mx, double my, CovarMatrix coar)
{
    // pos_x_ = pos_x;
    // pos_y_ = pos_y;
    // vel_x_ = vel_x;
    // vel_y_ = vel_y;
    // sigma_1_ = sigvx;
    // sigma_2_ = sigvy;
    // rho_ = 0;

    // pre-calculate intermediate values
    // coeff1_ = std::sqrt(2 * M_PI * sigma_1_ * sigma_1_);
    // coeff2_ = -(2 * sigma_1_ * sigma_1_);
}

double BiGaussianDistribution::operator()(double x, double y)
{
    //// transform position to align with velocity vector
    // double theta = std::atan2(vel_y_, vel_x_);
    // double x_hat = (x - pos_x_) * std::cos(theta) + (y - pos_y_) * std::sin(theta);
    // double y_hat = -(x - pos_x_) * std::sin(theta) + (y - pos_y_) * std::cos(theta);

    // double x_err = x_hat;
    // double y_err = y_hat;

    // double val = 0;
    // if (std::abs(y_err) < 1.8)
    //     // val = std::exp(x_err * x_err / coeff2_ + x_err * y_err * coeff4_ + y_err * y_err / coeff3_) / coeff1_;
    //     val = std::exp(x_err * x_err / coeff2_) / coeff1_;

    // return val;
}
