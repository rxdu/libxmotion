/* 
 * threat_map.cpp
 * 
 * Created on: Dec 14, 2018 05:05
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/threat_map.hpp"

#include <cmath>
#include <iostream>

namespace librav
{
ThreatMap::ThreatMap(double px, double py) : cx_(px),
                                             cy_(py)
{
}

/* 
 * Reference:
 *  [1] https://en.wikipedia.org/wiki/Gaussian_function
 *  [2] https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance
 *  [3] http://mathworld.wolfram.com/BivariateNormalDistribution.html
 *  [4] https://math.stackexchange.com/questions/162051/how-to-rotate-a-gaussian
 */
double ThreatMap::operator()(double x, double y, double vx, double vy)
{
    Eigen::Matrix<double, 2, 1> pe;
    pe << x - cx_, y - cy_;
    // pe << std::log(x) - std::log(cx_), std::log(y) - std::log(cy_);

    Eigen::Matrix<double, 2, 1> vvec;
    vvec << vx, vy;

    double theta = -std::atan2(vy, vx);

    Omega_(0, 0) = std::cos(theta) * std::cos(theta) / (2 * sigma_f * sigma_f) +
                   std::sin(theta) * std::sin(theta) / (2 * sigma_s * sigma_s);
    Omega_(0, 1) = -std::sin(2 * theta) / (4 * sigma_f * sigma_f) +
                   std::sin(2 * theta) / (4 * sigma_s * sigma_s);
    Omega_(1, 0) = Omega_(0, 1);
    Omega_(1, 1) = std::sin(theta) * std::sin(theta) / (2 * sigma_f * sigma_f) +
                   std::cos(theta) * std::cos(theta) / (2 * sigma_s * sigma_s);

    double res1 = -pe.transpose() * Omega_ * pe;
    double res2 = -alpha * vvec.transpose() * pe;

    // std::cout << "theta: " << theta << std::endl;
    // std::cout << "-----------" << std::endl;
    // std::cout << "res1: " << res1 << std::endl;
    // std::cout << "res2: " << res2 << std::endl;

    // return std::exp(res1);// / (x * y);
    // return std::exp(res1) / (1.0 + std::exp(res2));
    // return std::exp(res1) * (1 + 2.0 / M_PI * std::atan(-res2 * 0.5));
    return - ((x - cx_) * (x - cx_) + (y - cy_) * (y - cy_));
}

/* bivariate lognormal */
// double ThreatMap::operator()(double x, double y, double vx, double vy)
// {
//     Eigen::Matrix<double, 2, 1> pe;
//     pe << std::log(x) - std::log(cx_), std::log(y) - std::log(cy_);

//     Eigen::Matrix<double, 2, 1> vvec;
//     vvec << vx, vy;

//     double theta = -std::atan2(vy, vx);

//     Omega_(0, 0) = std::cos(theta) * std::cos(theta) / (2 * sigma_f * sigma_f) +
//                    std::sin(theta) * std::sin(theta) / (2 * sigma_s * sigma_s);
//     Omega_(0, 1) = -std::sin(2 * theta) / (4 * sigma_f * sigma_f) +
//                    std::sin(2 * theta) / (4 * sigma_s * sigma_s);
//     Omega_(1, 0) = Omega_(0, 1);
//     Omega_(1, 1) = std::sin(theta) * std::sin(theta) / (2 * sigma_f * sigma_f) +
//                    std::cos(theta) * std::cos(theta) / (2 * sigma_s * sigma_s);

//     double res1 = -pe.transpose() * Omega_ * 100 * pe;
//     double res2 = -alpha * vvec.transpose() * pe;

//     // std::cout << "theta: " << theta << std::endl;
//     // std::cout << "-----------" << std::endl;
//     // std::cout << "res1: " << res1 << std::endl;
//     // std::cout << "res2: " << res2 << std::endl;

//     return std::exp(res1);
// }

/* skewed bivariate Gaussian */
// double ThreatMap::operator()(double x, double y, double vx, double vy)
// {
//     Eigen::Matrix<double, 2, 1> pe;
//     pe << x - cx_, y - cy_;

//     Eigen::Matrix<double, 2, 1> vvec;
//     vvec << vx, vy;

//     double theta = -std::atan2(vy, vx);

//     Omega_(0, 0) = std::cos(theta) * std::cos(theta) / (2 * sigma_f * sigma_f) +
//                    std::sin(theta) * std::sin(theta) / (2 * sigma_s * sigma_s);
//     Omega_(0, 1) = -std::sin(2 * theta) / (4 * sigma_f * sigma_f) +
//                    std::sin(2 * theta) / (4 * sigma_s * sigma_s);
//     Omega_(1, 0) = Omega_(0, 1);
//     Omega_(1, 1) = std::sin(theta) * std::sin(theta) / (2 * sigma_f * sigma_f) +
//                    std::cos(theta) * std::cos(theta) / (2 * sigma_s * sigma_s);

//     double res1 = -pe.transpose() * Omega_ * pe;
//     double res2 = -alpha * vvec.transpose() * pe;

//     // std::cout << "theta: " << theta << std::endl;
//     // std::cout << "-----------" << std::endl;
//     // std::cout << "res1: " << res1 << std::endl;
//     // std::cout << "res2: " << res2 << std::endl;

//     return std::exp(res1) / (1.0 + std::exp(res2));
// }

void ThreatMap::PrintInfo()
{
    std::cout << "center position: " << cx_ << " , " << cy_ << std::endl;
}

} // namespace librav