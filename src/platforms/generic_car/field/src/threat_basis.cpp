/* 
 * threat_basis.cpp
 * 
 * Created on: Jan 23, 2018 13:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "field/threat_basis.hpp"

using namespace librav;

ThreatBasis::ThreatBasis(int64_t size_x, int64_t size_y) : ScalarField(size_x, size_y)
{
}

void ThreatBasis::UpdateThreatBasis(std::function<double(double, double)> dist_func)
{
    for (int64_t i = 0; i < size_x_; ++i)
        for (int64_t j = 0; j < size_y_; ++j)
            SetValueAtCoordinate(i, j, dist_func(i, j));
}