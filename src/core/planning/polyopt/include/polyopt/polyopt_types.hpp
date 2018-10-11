/* 
 * polyopt_types.hpp
 * 
 * Created on: Apr 03, 2018 13:25
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYOPT_TYPES_HPP
#define POLYOPT_TYPES_HPP

#include <vector>
#include <cstdint>
#include <iostream>

#include "common/poly_curve.hpp"

namespace librav
{
struct OptResultCurve
{
    std::vector<CurveParameter> segments;
    double cost;

    void print()
    {
        std::cout << "\n**************************************\n"
                  << std::endl;
        std::cout << "cost: " << cost << std::endl;

        uint32_t idx = 0;
        for (auto &seg : segments)
        {
            std::cout << "\nseg " << idx << " : " << std::endl;
            uint32_t coeff_idx = 0;
            for (auto &coef : seg.coeffs)
                std::cout << coeff_idx++ << " : " << coef << std::endl;
            std::cout << "start time: " << seg.ts << " , end time: " << seg.te << std::endl;
            idx++;
        }
        std::cout << "\n**************************************\n"
                  << std::endl;
    }
};

struct OptResultParam
{
    std::vector<double> params;
    double cost;
};
}

#endif /* POLYOPT_TYPES_HPP */
