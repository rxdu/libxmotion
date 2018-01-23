/* 
 * threat_distribution.hpp
 * 
 * Created on: Jan 23, 2018 13:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_DISTRIBUTION_HPP
#define THREAT_DISTRIBUTION_HPP

#include <cstdint>

namespace librav
{

class GaussianThreat
{
public:
  GaussianThreat() = default;
  GaussianThreat(double miu1, double miu2, double sigma);

  void SetParameters(double miu1, double miu2, double sigma);
  double operator()(double x, double y);

private:
  double miu1_ = 0;
  double miu2_ = 0;
  double sigma_ = 1;

  double coeff1_ = 0;
  double coeff2_ = 1;
};
}

#endif /* THREAT_DISTRIBUTION_HPP */
