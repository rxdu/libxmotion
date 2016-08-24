/*
 * test_utils.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#include <iostream>

#include "gurobi_c++.h"

#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_utils.h"

using namespace std;
using namespace srcl_ctrl;
using namespace Eigen;

int main(int   argc, char *argv[])
{
	PolyOptUtils po_utils;

//	po_utils.GetDerivativeCoeffs(7, 0);
//	po_utils.GetDerivativeCoeffs(7, 1);
//	po_utils.GetDerivativeCoeffs(7, 2);
//	po_utils.GetDerivativeCoeffs(7, 3);

	PolynomialCoeffs coeff(8);
	po_utils.GetDerivativeCoeffs(7, 0, coeff);
	std::cout << coeff << std::endl;
	po_utils.GetDerivativeCoeffs(7, 1, coeff);
	std::cout << coeff << std::endl;
	po_utils.GetDerivativeCoeffs(7, 2, coeff);
	std::cout << coeff << std::endl;
	po_utils.GetDerivativeCoeffs(7, 3, coeff);
	std::cout << coeff << std::endl;
	po_utils.GetDerivativeCoeffs(7, 4, coeff);
	std::cout << coeff << std::endl;
}

