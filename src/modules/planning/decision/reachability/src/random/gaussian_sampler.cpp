/* 
 * gaussian_sampler.cpp
 * 
 * Created on: Mar 22, 2018 18:08
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "random/gaussian_sampler.hpp"

using namespace robotnav;

GaussianSampler::GaussianSampler(double mean, double variance) : mean_(mean), variance_(variance), distribution_(mean, variance)
{
}

void GaussianSampler::Sample(double *val)
{
    *val = distribution_(generator_);
}