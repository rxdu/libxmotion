/* 
 * poly_curve.cpp
 * 
 * Created on: Jun 08, 2016
 * Description:   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "polyopt/poly_curve.hpp"

using namespace librav;

void PolynomialMath::GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs)
{
	{
		int64_t N = poly_order;
		int64_t r = deriv_order;

		coeffs = Eigen::ArrayXXf::Ones(1, N + 1);

		/* if deriv_order == 0, no derivative is taken */
		if (r == 0)
			return;

		/* otherwise calculate derivative coefficients */
		int64_t n;
		for (n = 0; n <= N; n++)
		{
			if (n < r)
				coeffs(0, N - n) = 0;
			else
			{
				uint64_t multiply = 1;
				for (uint64_t m = 0; m <= r - 1; m++)
				{
					multiply *= (n - m);
				}
				coeffs(0, N - n) = multiply;
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
	for (int i = 0; i <= N; i++)
	{
		double item_val;

		if (N - i >= r)
		{
			item_val = deriv_coeff[i] * coeffs[i] * std::pow(tau, N - i - r);

			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}

PolyCurve::PolyCurve(const std::vector<double> &coefficients, bool coeff_nondim, double start_t, double end_t, std::string str) : is_nondim_(coeff_nondim)
{
	for (auto &coeff : coefficients)
		param_.coeffs.push_back(coeff);
	param_.ts = start_t;
	param_.te = end_t;
	name_ = str;
}

double PolyCurve::GetCurvePointDerivVal(uint32_t deriv, double t)
{
	int64_t N = param_.coeffs.size() - 1;
	int64_t r = deriv;
	double ts = GetRefactoredTime(t);
	PolynomialMath::PolynomialCoeffs deriv_coeff(N + 1);
	PolynomialMath::GetDerivativeCoeffs(N, r, deriv_coeff);

	double val = 0;
	for (int i = 0; i <= N; i++)
	{
		double item_val;

		if (N - i >= r)
		{
			item_val = deriv_coeff[i] * param_.coeffs[i] * std::pow(ts, N - i - r);
			//std::cout << "deriv_coeff: " << deriv_coeff[i] << " ; coeff: " << coeffs_[i] << " ; power: " << N-i-r << " ; val: " << item_val << std::endl;
		}
		else
			item_val = 0;

		val += item_val;
	}

	return val;
}

double PolyCurve::GetRefactoredTime(double t)
{
	if (t < param_.ts)
		t = param_.ts;
	if (t > param_.te)
		t = param_.te;

	if (is_nondim_)
		return (t - param_.ts) / (param_.te - param_.ts);
	else
		return t;
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
	for (auto &coef : param_.coeffs)
		std::cout << coeff_idx++ << " : " << coef << std::endl;
	std::cout << "start time: " << param_.ts << " , end time: " << param_.te << std::endl;
}
