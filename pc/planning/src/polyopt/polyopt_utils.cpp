/*
 * polyopt_utils.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#include <iostream>

#include "polyopt/polyopt_utils.h"

using namespace srcl_ctrl;
using namespace Eigen;

void PolyOptUtils::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	int32_t n,r,N;

	r = deriv_order;
	N = poly_order;

	coeffs = ArrayXXf::Ones(1, N+1);

	/* if deriv_order == 0, no derivative is taken */
	if(r == 0)
		return;

	/* otherwise calculate derivative coefficients */
	for(n = 0; n <= N; n++)
	{
		if(n < r)
			coeffs(0, N - n) = 0;
		else
		{
			uint32_t multiply = 1;
			for(uint32_t m = 0; m <= r - 1; m++)
			{
				multiply *= (n - m);
			}
			coeffs(0,N - n) = multiply;
		}
	}
}


