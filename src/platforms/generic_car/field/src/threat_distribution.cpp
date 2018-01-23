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

GaussianThreat::GaussianThreat(double miu1, double miu2, double sigma) : miu1_(miu1),
                                                                         miu2_(miu2),
                                                                         sigma_(sigma)
{
    // update coeff1_ and coeff2_
    SetParameters(miu1, miu2, sigma);
}

void GaussianThreat::SetParameters(double miu1, double miu2, double sigma)
{
    miu1_ = miu1;
    miu2_ = miu2;
    sigma_ = sigma;

    coeff1_ = (2 * M_PI * sigma_ * sigma_);
    coeff2_ = -(2 * sigma_ * sigma_);
}

double GaussianThreat::operator()(double x, double y)
{
    double x_err = x - miu1_;
    double y_err = y - miu2_;

    double val = std::exp((x_err * x_err + y_err * y_err) / coeff2_) / coeff1_;

    return val;
}