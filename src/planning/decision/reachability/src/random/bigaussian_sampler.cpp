/* 
 * bigaussian_sampler.cpp
 * 
 * Created on: Mar 21, 2018 16:22
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "xmnavigation/random/bigaussian_sampler.hpp"

#include <cmath>

using namespace xmotion;

BiGaussianSampler::BiGaussianSampler(double sigma_x, double sigma_y, double rho) : sigma_x_(sigma_x),
                                                                            sigma_y_(sigma_y),
                                                                            rho_(rho)
{
}

void BiGaussianSampler::Sample(double *x, double *y)
{
    // Standard bivariate-gaussian construction (same algorithm as GSL's
    // gsl_ran_bivariate_gaussian): correlate two unit normals via rho.
    double u = unit_normal_(generator_);
    double v = unit_normal_(generator_);
    *x = sigma_x_ * u;
    *y = sigma_y_ * (rho_ * u + std::sqrt(1.0 - rho_ * rho_) * v);
}
