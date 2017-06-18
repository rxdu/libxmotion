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

struct  CurveParameter{
	// coefficients arranged from high order to low order
	std::vector<double> coeffs;
	double ts;
	double te;
};

class PolyCurve {
public:
	PolyCurve():is_nondim_(false){};
	PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t);
	PolyCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t, std::string str);
	~PolyCurve()=default;

private:
	bool is_nondim_;

public:
	CurveParameter param_;
	std::string name_;

private:
	double GetCurvePointDerivVal(uint32_t deriv, double t);

public:
	void SetCurveName(std::string str){ name_ = str; };
	std::string GetCurveName(){ return name_; };

	double GetRefactoredTime(double t);

	double GetCurvePointPos(double t);
	double GetCurvePointVel(double t);
	double GetCurvePointAcc(double t);

	void print();
};

}

#endif /* COMMON_POLY_CURVE_H_ */
