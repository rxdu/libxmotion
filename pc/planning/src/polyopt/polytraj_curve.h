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

typedef struct {
	// coefficients arranged from high order to low order
	std::vector<double> coeffs;
	double ts;
	double te;
} CurveParameter;

class PolyTrajCurve {
public:
	PolyTrajCurve();
	PolyTrajCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t);
	PolyTrajCurve(const std::vector<double>& coefficients, bool coeff_nondim, double start_t, double end_t, std::string str);
	~PolyTrajCurve(){};

private:
	bool is_nondim_;

public:
	CurveParameter param_;
	std::string name_;

private:
	double GetCurvePointDerivVal(uint32_t deriv, double t);

public:
	void SetCurveName(std::string str){ name_ = str; };
	std::string SetCurveName(){ return name_; };

	double GetCurvePointPos(double t);
	double GetCurvePointVel(double t);
	double GetCurvePointAcc(double t);

	double GetRefactoredTime(double t);

	void print();
};

}

#endif /* PLANNING_SRC_POLYOPT_POLYTRAJ_CURVE_H_ */
