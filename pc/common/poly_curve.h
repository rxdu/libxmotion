/*
 * poly_curve.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#ifndef COMMON_POLY_CURVE_H_
#define COMMON_POLY_CURVE_H_

#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include "common/poly_helper.h"

namespace srcl_ctrl {

typedef struct {
	// coefficients arranged from high order to low order
	std::vector<double> coeffs;
	double ts;
	double te;
} CurveParameter;

class PolyCurve {
public:
	PolyCurve():is_nondim_(false){}

	PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t):
			is_nondim_(coeff_nondim)
	{
		for(auto& coeff:coefficients)
			param_.coeffs.push_back(coeff);
		param_.ts = start_t;
		param_.te = end_t;
	}

	PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t, std::string str):
			is_nondim_(coeff_nondim)
	{
		for(auto& coeff:coefficients)
			param_.coeffs.push_back(coeff);
		param_.ts = start_t;
		param_.te = end_t;
		name_ = str;
	}

	~PolyCurve()=default;

private:
	bool is_nondim_;

public:
	CurveParameter param_;
	std::string name_;

private:
	double GetCurvePointDerivVal(uint32_t deriv, double t)
	{
		int64_t N = param_.coeffs.size() - 1;
		int64_t r = deriv;
		double ts = GetRefactoredTime(t);
		PolyHelper::PolynomialCoeffs deriv_coeff(N + 1);
		PolyHelper::GetDerivativeCoeffs(N, r, deriv_coeff);

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

public:
	void SetCurveName(std::string str){ name_ = str; };
	std::string GetCurveName(){ return name_; };

	double GetRefactoredTime(double t)
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

	double GetCurvePointPos(double t)
	{
		double ts = GetRefactoredTime(t);

		return GetCurvePointDerivVal(0, ts);
	}

	double GetCurvePointVel(double t)
	{
		double ts = GetRefactoredTime(t);

		return GetCurvePointDerivVal(1, ts);
	}

	double GetCurvePointAcc(double t)
	{
		double ts = GetRefactoredTime(t);

		return GetCurvePointDerivVal(2, ts);
	}

	void print()
	{
		uint32_t coeff_idx = 0;
		std::cout << "\ncurve " << name_ << std::endl;
		for(auto& coef:param_.coeffs)
			std::cout << coeff_idx++ << " : " << coef << std::endl;
		std::cout << "start time: " << param_.ts << " , end time: " << param_.te << std::endl;
	}
};

}

#endif /* COMMON_POLY_CURVE_H_ */
