/* 
 * threat_map.hpp
 * 
 * Created on: Dec 14, 2018 05:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_MAP_HPP
#define THREAT_MAP_HPP

#include <eigen3/Eigen/Core>

namespace librav
{
class ThreatMap
{
  public:
    ThreatMap() = default;
    ThreatMap(double px, double py);

    const double alpha = 0.05;
    const double sigma_f = (4.8 / 2) * (4.8 / 2);
    const double sigma_s = (1.8 * 4 / 5) * (1.8 * 4 / 5);

    double operator()(double x, double y, double vx, double vy);
    void PrintInfo();

  private:
    double cx_ = 0.0;
    double cy_ = 0.0;

    Eigen::Matrix<double, 2, 2> Omega_;
};
} // namespace librav

#endif /* THREAT_MAP_HPP */
