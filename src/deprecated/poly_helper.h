/*
 * librav_math.h
 *
 *  Created on: June 08, 2017
 *      Author: rdu
 */

#ifndef COMMON_POLY_HELPER_H_
#define COMMON_POLY_HELPER_H_

#include <cstdint>
#include <cmath>
#include <vector>

#include "eigen3/Eigen/Core"

namespace librav
{
namespace PolynomialMath
{

using PolynomialCoeffs = Eigen::Array<float,1, Eigen::Dynamic>;

void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs);
double GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau);

}
}

#endif /* COMMON_POLY_HELPER_H_ */
