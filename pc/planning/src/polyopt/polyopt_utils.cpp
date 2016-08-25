/*
 * polyopt_utils.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cmath>
#include <limits>

#include "polyopt/polyopt_utils.h"

using namespace srcl_ctrl;
using namespace Eigen;

void PolyOptUtils::GetDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1,
		Eigen::Ref<Eigen::MatrixXf> q)
{
	int64_t N = poly_order;
	int64_t r = deriv_order;

	q = MatrixXf::Zero(N+1, N+1);

	for(int64_t i = 0; i <= N; i++)
		for(int64_t j = 0; j <= N; j++)
		{
			if(i >= r && j >= r)
			{
				uint64_t multiply = 1;
				uint64_t order = i + j - 2 * r + 1;

				for(int64_t m = 0; m <= r - 1; m++)
				{
					multiply *= (i - m) * (j - m);
				}

				//q(i,j) = 2.0 * multiply * (std::pow(t1, order) - std::pow(t0, order))/order;
				q(i,j) = multiply * (std::pow(t1, order) - std::pow(t0, order))/order;
			}
		}
}

void PolyOptUtils::GetNonDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1,
		Eigen::Ref<Eigen::MatrixXf> q)
{
	GetDimQMatrix(poly_order, deriv_order, 0.0, 1.0, q);

	double nondim_coeff;
	nondim_coeff = 1.0/(std::pow((t1 - t0), 2*deriv_order - 1));

	q = nondim_coeff * q;
}

void PolyOptUtils::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	int64_t N = poly_order;
	int64_t r = deriv_order;

	coeffs = ArrayXXf::Ones(1, N+1);

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

uint32_t PolyOptUtils::GetNonZeroCoeffNum(const Eigen::Ref<const PolynomialCoeffs> coeffs)
{
	uint32_t order = 0;

	PolynomialCoeffs data = coeffs;
	for(int32_t i = 0; i < data.cols(); i++)
		if(data(i) != 0) order++;

	return order;
}

void PolyOptUtils::GetNonDimEqualityConstrs(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts)
{
//	std::cout << "key frames: \n" << keyframe_vals << std::endl;
//	std::cout << "time stamps: \n" << keyframe_ts << std::endl;

	// TODO check if size of keyframe values and that of keyframe time stamps match
	// TODO check if keyframe_num is greater than 1

	int64_t N = poly_order;
	int64_t r = deriv_order;
	int64_t last_keyframe_idx = keyframe_num - 1;

	MatrixXf A_eq = MatrixXf::Zero(2 * r, (keyframe_num - 1) * (N + 1));
	MatrixXf b_eq = MatrixXf::Zero(2 * r, 1);

	// check each key frame
	for(int64_t j = 0; j <= last_keyframe_idx; j++)
	{
		// check each derivative of one key frame
		for(int64_t i = 0; i <= r-1; i++)
		{
			// only one constraint at the first and last key frame
			if(j == 0)
			{
				b_eq(j*2 + i,0) = keyframe_vals(i, j);
			}
			else if(j == last_keyframe_idx)
			{
				b_eq(j*2 + i,0) = keyframe_vals(i, j);
			}
			// two constraints at the intermediate key frames
			else
			{
				// special case: if no constraint is specified, just ensure continuity
				if(keyframe_vals(i, j) == std::numeric_limits<float>::infinity())
				{

					b_eq(j*2 + i,0) = 0;
				}
				else {

					b_eq(j*2 + i*2,0) = keyframe_vals(i, j);
					b_eq(j*2 + i*2 + 1,0) = keyframe_vals(i, j);
				}
			}
		}
	}

	std::cout << "A_eq:\n" << A_eq << std::endl;
	std::cout << "b_eq:\n" << b_eq << std::endl;
}
