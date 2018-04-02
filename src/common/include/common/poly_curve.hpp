/* 
 * poly_curve.hpp
 * 
 * Created on: Aug 29, 2016
 * Description:   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef POLY_CURVE_HPP
#define POLY_CURVE_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include "eigen3/Eigen/Core"

namespace librav
{

namespace PolynomialMath
{
using PolynomialCoeffs = Eigen::Array<float, 1, Eigen::Dynamic>;

void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs);
double GetPolynomialValue(std::vector<double> coeffs, uint32_t deriv_order, double tau);
}

struct CurveParameter
{
	// coefficients arranged from high order to low order
	std::vector<double> coeffs;
	double ts;
	double te;
};

class PolyCurve
{
  public:
	PolyCurve(){};
	PolyCurve(const std::vector<double> &coefficients, bool coeff_nondim, double start_t, double end_t, std::string str = "default");
	~PolyCurve() = default;

	CurveParameter param_;
	std::string name_;

	void SetCurveName(std::string str) { name_ = str; };
	std::string GetCurveName() { return name_; };

	double GetRefactoredTime(double t);

	double GetCurvePointPos(double t);
	double GetCurvePointVel(double t);
	double GetCurvePointAcc(double t);

	void print();

  private:
	bool is_nondim_ = false;
	double GetCurvePointDerivVal(uint32_t deriv, double t);
};
}

#endif /* POLY_CURVE_HPP */
