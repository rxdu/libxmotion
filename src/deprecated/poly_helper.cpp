/*
 * poly_helper.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: rdu
 */

#include "common/librav_math.h"

using namespace librav;

void PolynomialMath::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	{
		int64_t N = poly_order;
		int64_t r = deriv_order;

		coeffs = Eigen::ArrayXXf::Ones(1, N+1);

		/* if deriv_order == 0, no derivative is taken */
		if(r == 0)
			return;

		/* otherwise calculate derivative coefficients */
		int64_t n;
		for(n = 0; n <= N; n++)
		{
			if(n < r)
				coeffs(0, N - n) = 0;
			else
			{
				uint64_t multiply = 1;
				for(uint64_t m = 0; m <= r - 1; m++)
				{
					multiply *= (n - m);
				}
				coeffs(0,N - n) = multiply;
			}
		}
	}
}

double PolynomialMath::GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau)
{
	double val = 0;
	int64_t N = coeffs.size() - 1;
	int64_t r = deriv_order;

	PolynomialCoeffs deriv_coeff(N + 1);
	PolynomialMath::GetDerivativeCoeffs(N, r, deriv_coeff);

	uint32_t coeff_size = coeffs.size();
	for(int i = 0; i <= N; i++)
	{
		double item_val;

		if(N - i >= r) {
			item_val =  deriv_coeff[i] * coeffs[i] * std::pow(tau, N - i - r);

			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}


