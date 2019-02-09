/* 
 * uniform_sampler.hpp
 * 
 * Created on: Feb 03, 2019 06:40
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef UNIFORM_SAMPLER_HPP
#define UNIFORM_SAMPLER_HPP

#include <random>

namespace librav
{
class UniformSampler
{
public:
  UniformSampler(double min, double max);
  ~UniformSampler() = default;

  void Sample(double *val);

private:
  double min_;
  double max_;

  std::random_device rd_{};
  std::mt19937 generator_{rd_()};
  std::uniform_real_distribution<> distribution_;
};
}

#endif /* UNIFORM_SAMPLER_HPP */
