/* 
 * field_basis.cpp
 * 
 * Created on: Dec 14, 2018 03:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "field_decomp/field_basis.hpp"

#include <cmath>
#include <iostream>

namespace librav
{
FieldBasis::FieldBasis(double px, double py) : cx_(px),
                                               cy_(py)
{
}

double FieldBasis::operator()(double x, double y)
{
    // Eigen::Matrix<double, 2, 1> pe;
    // pe << x - cx_, y - cy_;

    // theta_ = std::atan2(vy, vx);

    // std::cout << "theta: " << theta_ << std::endl;

    // Omega_(0, 0) = std::cos(theta_) * std::cos(theta_) / (2 * sigma_f * sigma_f) +
    //                std::sin(theta_) * std::sin(theta_) / (2 * sigma_s * sigma_s);
    // Omega_(0, 1) = -std::sin(2 * theta_) / (4 * sigma_f * sigma_f) +
    //                std::sin(2 * theta_) / (4 * sigma_s * sigma_s);
    // Omega_(1, 0) = Omega_(0, 1);
    // Omega_(1, 1) = std::sin(theta_) * std::sin(theta_) / (2 * sigma_f * sigma_f) +
    //                std::cos(theta_) * std::cos(theta_) / (2 * sigma_s * sigma_s);

    // v_vec_ << vx, vy;

    // double res1 = -pe.transpose() * Omega_ * pe;
    // double res2 = -alpha * v_vec_.transpose() * pe;

    // // std::cout << "-----------" << std::endl;
    // // std::cout << "res1: " << res1 << std::endl;
    // // std::cout << "res2: " << res2 << std::endl;

    // return std::exp(res1) / (1.0 + std::exp(res2));
    return 0;
}

void FieldBasis::PrintInfo()
{
    std::cout << "center position: " << cx_ << " , " << cy_ << std::endl;
}

} // namespace librav