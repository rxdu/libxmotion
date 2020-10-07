/* 
 * gaussian_sampler.hpp
 * 
 * Created on: Mar 22, 2018 17:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GAUSSIAN_SAMPLER_HPP
#define GAUSSIAN_SAMPLER_HPP

#include <random>

namespace autodrive
{
class GaussianSampler
{
public:
  GaussianSampler(double mean, double variance);
  ~GaussianSampler() = default;

  void Sample(double *val);

private:
  double mean_;
  double variance_;

  std::random_device rd_{};
  std::mt19937 generator_{rd_()};
  std::normal_distribution<> distribution_;
};
}

#endif /* GAUSSIAN_SAMPLER_HPP */
