/*
 * polytraj.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <iostream>

#include "polyopt/polyopt_math.h"
#include "polyopt/poly_curve.h"

using namespace srcl_ctrl;

PolyCurve::PolyCurve():
		is_nondim_(false)
{

}

PolyCurve::PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t):
		is_nondim_(coeff_nondim)
{
	for(auto& coeff:coefficients)
		param_.coeffs.push_back(coeff);
	param_.ts = start_t;
	param_.te = end_t;
}

PolyCurve::PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t, std::string str):
		is_nondim_(coeff_nondim)
{
	for(auto& coeff:coefficients)
		param_.coeffs.push_back(coeff);
	param_.ts = start_t;
	param_.te = end_t;
	name_ = str;
}

double PolyCurve::GetRefactoredTime(double t)
{
	if(t < param_.ts)
		t = param_.ts;
	if(t > param_.te)
		t = param_.te;

	if(is_nondim_)
		return (t - param_.ts) / (param_.te - param_.ts);
	else
		return t;
}

double PolyCurve::GetCurvePointDerivVal(uint32_t deriv, double t)
{
	int64_t N = param_.coeffs.size() - 1;
	int64_t r = deriv;
	double ts = GetRefactoredTime(t);
	PolynomialCoeffs deriv_coeff(N + 1);
	PolyOptMath::GetDerivativeCoeffs(N, r, deriv_coeff);

	double val = 0;
	for(int i = 0; i <= N; i++)
	{
		double item_val;

		if(N - i >= r) {
			item_val =  deriv_coeff[i] * param_.coeffs[i] * std::pow(ts, N - i - r);
			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs_[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}

double PolyCurve::GetCurvePointPos(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(0, ts);
}

double PolyCurve::GetCurvePointVel(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(1, ts);
}

double PolyCurve::GetCurvePointAcc(double t)
{
	double ts = GetRefactoredTime(t);

	return GetCurvePointDerivVal(2, ts);
}

void PolyCurve::print()
{
	uint32_t coeff_idx = 0;
	std::cout << "\ncurve " << name_ << std::endl;
	for(auto& coef:param_.coeffs)
		std::cout << coeff_idx++ << " : " << coef << std::endl;
	std::cout << "start time: " << param_.ts << " , end time: " << param_.te << std::endl;
}
