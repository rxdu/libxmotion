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

void PolyOptUtils::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<Eigen::ArrayXXf>& coeffs)
{
	int32_t n,r,N;

	r = deriv_order;
	N = poly_order;

	ArrayXXf coeffs_l = ArrayXXf::Ones(1, N+1);

	coeffs = coeffs_l;

	// if deriv_order == 0, no derivative is taken
	if(r == 0)
	{
//		std::cout << "coefficients: " << std::endl;
//		std::cout << coeffs << std::endl;

		return;
	}

	// otherwise calculate derivative coefficients
	for(n = N; n >= 0; n--)
	{
		if(n < r)
			coeffs_l(0,N - n) = 0;
		else
		{
			uint32_t multiply = 1;
			for(uint32_t m = 0; m <= r - 1; m++)
			{
				multiply *= (n - m);
			}
			coeffs_l(0,N - n) = multiply;
		}

		coeffs = coeffs_l;
	}

//	std::cout << "coefficients: " << std::endl;
//	std::cout << coeffs << std::endl;
}


