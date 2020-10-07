/* 
 * uniform_sampler.cpp
 * 
 * Created on: Feb 03, 2019 06:41
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "random/uniform_sampler.hpp"

using namespace autodrive;

UniformSampler::UniformSampler(double min, double max) : min_(min), max_(max), distribution_(min, max)
{
}

void UniformSampler::Sample(double *val)
{
    *val = distribution_(generator_);
}