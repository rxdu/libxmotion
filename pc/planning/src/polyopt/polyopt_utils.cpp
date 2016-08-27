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

	// flip Q so that the coefficients are arranged from sigma_n to sigma_0
	Eigen::MatrixXf q_flip = q;
	uint64_t row_size = q_flip.cols();
	uint64_t col_size = q_flip.rows();
	q_flip = Eigen::MatrixXf::Zero(row_size, col_size);

	for(int64_t i = 0; i < row_size; i++)
		for(int64_t j = 0; j < col_size; j++)
			q_flip(i,j) = q(row_size - 1 - i, col_size - 1 - j);

	q = q_flip;
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
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_eq, Eigen::Ref<Eigen::MatrixXf> b_eq)
{
	// TODO check if size of keyframe values and that of keyframe time stamps match
	// TODO check if keyframe_num is greater than 1

	int64_t N = poly_order;
	int64_t r = deriv_order;
	int64_t traj_seg_num = keyframe_num - 1;

//	MatrixXf A_eq = MatrixXf::Zero(2 * r, (keyframe_num - 1) * (N + 1));
//	MatrixXf b_eq = MatrixXf::Zero(2 * r, 1);
	A_eq = MatrixXf::Zero(2 * r, (keyframe_num - 1) * (N + 1));
	b_eq = MatrixXf::Zero(2 * r, 1);

	// check each piece of trajectory
	for(int64_t j = 0; j < traj_seg_num; j++)
	{
		// check each derivative
		for(int64_t i = 0; i <= r-1; i++)
		{
			PolynomialCoeffs coeff(N + 1);
			GetDerivativeCoeffs(poly_order, i, coeff);

			// if there is only 1 segment, no continuity needs to be considered
			if(traj_seg_num == 1)
			{
				double nondim_coeff = 1/(std::pow(keyframe_ts(0, j+1)-keyframe_ts(0, j),i));
				for(int64_t k = 0; k < N + 1; k++)
				{
					// A(tau_0), A(tau_1)
					if(N - k >= i)
					{
						A_eq(j*2*r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(0, N - k - i );
						A_eq(j*2*r + r + i, j*(N + 1) + k) = nondim_coeff * coeff(k)*std::pow(1, N - k - i );
					}
					else
					{
						A_eq(j*2*r + i, j*(N + 1) + k) = 0;
						A_eq(j*2*r + r + i, j*(N + 1) + k) = 0;
					}
				}

				b_eq(j*2*r + i,0) = keyframe_vals(i, j);
				b_eq(j*2*r + r + i,0) = keyframe_vals(i, j + 1);
			}
			// if there are more segments, intermediate key frames need to be continuous
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

//	std::cout << "A_eq:\n" << A_eq << std::endl;
//	std::cout << "b_eq:\n" << b_eq << std::endl;
}
