/* 
 * bigaussian_sampler.cpp
 * 
 * Created on: Mar 21, 2018 16:22
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "random/bigaussian_sampler.hpp"

using namespace librav;

BiGaussianSampler::BiGaussianSampler(double sigma_x, double sigma_y, double rho) : sigma_x_(sigma_x),
                                                                            sigma_y_(sigma_y),
                                                                            rho_(rho)
{
    gsl_rng_env_setup();
    T_ = gsl_rng_default;
    r_ = gsl_rng_alloc(T_);
}

BiGaussianSampler::~BiGaussianSampler()
{
    gsl_rng_free(r_);
}

void BiGaussianSampler::Sample(double *x, double *y)
{
    gsl_ran_bivariate_gaussian(r_, sigma_x_, sigma_y_, rho_, x, y);
}