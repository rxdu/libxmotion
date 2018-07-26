/* 
 * threat_basis.cpp
 * 
 * Created on: Jan 23, 2018 13:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/threat_basis.hpp"

using namespace librav;

void ThreatBasis::SetCenterPosition(double x, double y)
{
    center_pos_x_ = x;
    center_pos_y_ = y;
}

void ThreatBasis::SetThreatBasisDistribution(std::function<double(double, double)> dist_func)
{
    for (int64_t i = 0; i < size_x_; ++i)
        for (int64_t j = 0; j < size_y_; ++j)
            SetValueAtCoordinate(i, j, dist_func(i, j));
}