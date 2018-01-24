/* 
 * threat_basis.hpp
 * 
 * Created on: Jan 23, 2018 13:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_BASIS_HPP
#define THREAT_BASIS_HPP

#include <functional>

#include "field/scalar_field.hpp"

namespace librav
{
class ThreatBasis : public ScalarField
{
  public:
    ThreatBasis(int64_t size_x = 0, int64_t size_y = 0): ScalarField(size_x, size_y){};
    
    void SetCenterPosition(double x, double y);
    void SetThreatBasisDistribution(std::function<double(double, double)> dist_func);

    double center_pos_x_;
    double center_pos_y_;
};
}

#endif /* THREAT_BASIS_HPP */
