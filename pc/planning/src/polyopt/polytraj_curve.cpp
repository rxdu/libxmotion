/*
 * polytraj.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <iostream>

#include "polyopt/polyopt_math.h"
#include "polyopt/polytraj_curve.h"

using namespace srcl_ctrl;

PolyTrajCurve::PolyTrajCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t):
		is_nondim_(coeff_nondim), t_start_(start_t), t_end_(end_t)
{
	for(auto& coeff:coefficients)
		coeffs_.push_back(coeff);
}

double PolyTrajCurve::GetRefactoredTime(double t)
{
	if(t < t_start_)
		t = t_start_;
	if(t > t_end_)
		t = t_end_;

	if(is_nondim_)
		return (t - t_start_) * (t_end_ - t_start_);
	else
		return t;
}

double PolyTrajCurve::GetCurvePointDerivVal(uint32_t deriv, double t)
{
	int64_t N = coeffs_.size() - 1;
	int64_t r = deriv;
	double ts = GetRefactoredTime(t);
	PolynomialCoeffs deriv_coeff(N + 1);
	PolyOptMath::GetDerivativeCoeffs(N, r, deriv_coeff);

	double val = 0;
	for(int i = 0; i <= N; i++)
	{
		double item_val;

		if(N - i >= r) {
			item_val =  deriv_coeff[i] * coeffs_[i] * std::pow(ts, N - i - r);
			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs_[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}

double PolyTrajCurve::GetCurvePointPos(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(0, ts);
}

double PolyTrajCurve::GetCurvePointVel(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(1, ts);
}

double PolyTrajCurve::GetCurvePointAcc(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(2, ts);
}
