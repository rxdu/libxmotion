/* 
 * gaussian_basis.hpp
 * 
 * Created on: Feb 17, 2019 09:04
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef GAUSSIAN_BASIS_HPP
#define GAUSSIAN_BASIS_HPP

#include <cmath>
#include <iostream>

#include "threat_field/radial_basis.hpp"

#include <eigen3/Eigen/Dense>

namespace librav
{
class GaussianBasis : RadialBasis
{
  public:
    GaussianBasis() = default;
    GaussianBasis(double cx, double cy) : cx_(cx), cy_(cy){};

    double operator()(double x, double y) override
    {
        Eigen::Matrix<double, 2, 1> xe;
        xe << x - cx_, y - cy_;

        return constant1_ * std::exp(-constant2_ * xe.transpose() * xe);
    }

    void PrintInfo() override
    {
        std::cout << "Gaussian basis center: " << cx_ << " , " << cy_ << std::endl;
    }

    static constexpr double sigma_n = 1;

  private:
    double cx_ = 0.0;
    double cy_ = 0.0;

    static constexpr double constant1_ = 1.0 / std::sqrt(2.0 * M_PI * sigma_n * sigma_n);
    static constexpr double constant2_ = 1.0 / (4.0 * sigma_n * sigma_n);
};
} // namespace librav

#endif /* GAUSSIAN_BASIS_HPP */
