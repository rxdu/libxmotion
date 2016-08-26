/*
 * test_qp.cpp
 *
 *  Created on: Aug 25, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cstdint>
#include <limits>

#include "gurobi_c++.h"

#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_utils.h"

using namespace std;
using namespace srcl_ctrl;
using namespace Eigen;

int main(int   argc, char *argv[])
{
	uint32_t r = 2;
	uint32_t N = 2 * r - 1;
	MatrixXf Q = MatrixXf::Zero(N+1, N+1);
	MatrixXf A_eq = MatrixXf::Zero(2 * r, 1 * (N + 1));
	MatrixXf b_eq = MatrixXf::Zero(2 * r, 1);

	MatrixXf keyframe_vals = MatrixXf::Zero(r, 2);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, 2);

	keyframe_vals(0,0) = -0.15;
	keyframe_vals(0,1) = 0.25;
	keyframe_vals(1,0) = 0.1;
	keyframe_vals(1,1) = 0.2;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;

	PolyOptUtils::GetNonDimQMatrix(N,r,0,1.2,Q);
	PolyOptUtils::GetNonDimEqualityConstrs(N, r, 2, keyframe_vals, keyframe_ts, A_eq, b_eq);

	std::cout << "\nQ: \n" << Q << std::endl;
	std::cout << "\nA_eq:\n" << A_eq << std::endl;
	std::cout << "\nb_eq:\n" << b_eq << std::endl;

	try {
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		// Create variables
		GRBVar sig0 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig0");
		GRBVar sig1 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig1");
		GRBVar sig2 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig2");
		GRBVar sig3 = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "sig3");

		// Integrate new variables
		model.update();

		// Set objective
		GRBQuadExpr obj = sig0*((125.0*sig0)/18.0 + (125.0*sig1)/36.0) + sig1*((125.0*sig0)/36.0 + (125.0*sig1)/54.0);
		model.setObjective(obj);

		// Add constraints
		model.addConstr(sig3 == -0.15, "c0");
		model.addConstr((5.0*sig2)/6.0 == 0, "c1");
		model.addConstr(sig0 + sig1 + sig2 + sig3 == 0.25, "c2");
		model.addConstr((5.0*sig0)/2.0 + (5.0*sig1)/3.0 + (5.0*sig2)/6.0 == 0, "c3");

		// Optimize model
		model.optimize();

		std::cout << sig0.get(GRB_StringAttr_VarName) << " "
				<< sig0.get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig1.get(GRB_StringAttr_VarName) << " "
				<< sig1.get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig2.get(GRB_StringAttr_VarName) << " "
				<< sig2.get(GRB_DoubleAttr_X) << std::endl;
		std::cout << sig3.get(GRB_StringAttr_VarName) << " "
				<< sig3.get(GRB_DoubleAttr_X) << std::endl;

		std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

	} catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	} catch(...) {
		cout << "Exception during optimization" << endl;
	}

	return 0;
}


