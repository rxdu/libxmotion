/* 
 * bigaussian_sampler.hpp
 * 
 * Created on: Mar 21, 2018 16:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BIGAUSSIAN_SAMPLER_HPP
#define BIGAUSSIAN_SAMPLER_HPP

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

namespace autodrive
{
class BiGaussianSampler
{
  public:
    BiGaussianSampler(double sigma_x, double sigma_y, double rho);
    ~BiGaussianSampler();

    void Sample(double *x, double *y);

  private:
    double sigma_x_;
    double sigma_y_;
    double rho_;

    const gsl_rng_type *T_;
    gsl_rng *r_;
};
}

#endif /* BIGAUSSIAN_SAMPLER_HPP */
