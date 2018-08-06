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

class GaussianPositionThreat
{
public:
  GaussianPositionThreat(double pos_x, double pos_y, double sigma1, double sigma2);

  void SetParameters(double pos_x, double pos_y, double sigma1, double sigma2);
  double operator()(double x, double y);

private:
  double pos_x_ = 0;
  double pos_y_ = 0;
  double sigma_1_ = 1;
  double sigma_2_ = 1;

  double coeff1_ = 0;
  double coeff2_ = 1;
  double coeff3_ = 1;
};

/*-------------------------------------------------------------*/

class GaussianPositionVelocityThreat
{
public:
  GaussianPositionVelocityThreat() = default;
  GaussianPositionVelocityThreat(double pos_x, double pos_y, double vel_x, double vel_y);

  void SetParameters(double pos_x, double pos_y, double vel_x, double vel_y, double sigp = 10, double sigv = 10);
  double operator()(double x, double y);

private:
  double pos_x_ = 0;
  double pos_y_ = 0;
  double vel_x_ = 0;
  double vel_y_ = 0;
  double sigma_1_ = 10;
  double sigma_2_ = 10;
  double rho_ = 0;

  double coeff1_ = 0;
  double coeff2_ = 1;
  double coeff3_ = 1;
  double coeff4_ = 0;
};

/*-------------------------------------------------------------*/

class BiasedGaussianThreat
{
public:
  BiasedGaussianThreat() = default;
  BiasedGaussianThreat(double pos_x, double pos_y, double vel_x, double vel_y);

  void SetParameters(double pos_x, double pos_y, double vel_x, double vel_y, double sigpx = 15, double sigpy = 15);
  double operator()(double x, double y);

private:
  double pos_x_ = 0;
  double pos_y_ = 0;
  double vel_x_ = 0;
  double vel_y_ = 0;
  double sigma_1_;
  double sigma_2_;
  double rho_ = 0;

  double coeff1_ = 0;
  double coeff2_ = 1;
  double coeff3_ = 1;
  double coeff4_ = 0;
};
} // namespace librav

#endif /* THREAT_DISTRIBUTION_HPP */
