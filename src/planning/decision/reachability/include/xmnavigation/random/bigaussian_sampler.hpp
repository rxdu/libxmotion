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

#include <random>

namespace xmotion
{
class BiGaussianSampler
{
  public:
    BiGaussianSampler(double sigma_x, double sigma_y, double rho);
    ~BiGaussianSampler() = default;

    void Sample(double *x, double *y);

  private:
    double sigma_x_;
    double sigma_y_;
    double rho_;

    std::random_device rd_{};
    std::mt19937 generator_{rd_()};
    std::normal_distribution<double> unit_normal_{0.0, 1.0};
};
}

#endif /* BIGAUSSIAN_SAMPLER_HPP */
