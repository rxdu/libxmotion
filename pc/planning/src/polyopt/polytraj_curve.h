/*
 * polytraj.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_POLYOPT_POLYTRAJ_CURVE_H_
#define PLANNING_SRC_POLYOPT_POLYTRAJ_CURVE_H_

#include <vector>
#include <string>
#include <cstdint>

namespace srcl_ctrl {

class PolyTrajCurve {
public:
	PolyTrajCurve();
	PolyTrajCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t);
	~PolyTrajCurve(){};

private:
	bool is_nondim_;

	// coefficients arranged from high order to low order
	std::vector<double> coeffs_;
	double t_start_;
	double t_end_;

	std::string name_;

private:
	double GetCurvePointDerivVal(uint32_t deriv, double t);
	double GetRefactoredTime(double t);

public:
	void SetCurveName(std::string str){ name_ = str; };
	double GetCurvePointPos(double t);
	double GetCurvePointVel(double t);
	double GetCurvePointAcc(double t);
};

}

#endif /* PLANNING_SRC_POLYOPT_POLYTRAJ_CURVE_H_ */
