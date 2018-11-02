/* 
 * gaussian_distribution.hpp
 * 
 * Created on: Jan 23, 2018 13:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GAUSSIAN_DISTRIBUTION_HPP
#define GAUSSIAN_DISTRIBUTION_HPP

#include <cstdint>

#include <eigen3/Eigen/Dense>

namespace librav
{
class GaussianDistribution
{
public:
  GaussianDistribution() = default;
  GaussianDistribution(double mean, double sd);

  void SetParameters(double m, double sd);

private:
  double mean_;
  double sd_;
};

class BiGaussianDistribution
{
public:
  using CovarMatrix = Eigen::Matrix<double, 2, 2>;

public:
  BiGaussianDistribution() = default;
  BiGaussianDistribution(double mx, double my, CovarMatrix covar);

  void SetParameters(double mx, double my, CovarMatrix coar);

  double GetMeanX() const { return mean_x_; }
  double GetMeanY() const { return mean_y_; }

  double operator()(double x, double y);

private:
  double mean_x_ = 0;
  double mean_y_ = 0;
  CovarMatrix covar_;

  double coeff1_ = 0;
  double coeff2_ = 1;
  double coeff3_ = 1;
};
} // namespace librav

#endif /* GAUSSIAN_DISTRIBUTION_HPP */
