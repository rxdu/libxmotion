/*
 * =====================================================================================
 *
 *       Filename:  qp_traj.cpp
 *
 *    Description:  Minimum snap trajectory optimization.
 *
 *        Version:  1.0
 *        Created:  08/23/2016 02:22:33 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ruixiang Du (rdu), ruixiang.du@gmail.com
 *   Organization:  Worcester Polytechnic Institute
 *
 * =====================================================================================
 */

#include "gurobi_c++.h"

using namespace std;

#include <stdlib.h>

int main(int argc, char *argv[]) {
    try {
	GRBEnv env = GRBEnv();

	GRBModel model = GRBModel(env);

	// create variables
	GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "x");
	GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "y");
	GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "z");

	model.update();

	// set objective
	GRBQuadExpr obj = x * x + x * y + y * y + y * z + z * z + 2 * x;
	model.setObjective(obj);

	// add constraint
	model.addConstr(x + 2 * y + 3 * z >= 4, "c0");
	model.addConstr(x + y >= 1, "c1");

	// optimize model
	model.optimize();

	cout << x.get(GRB_StringAttr_VarName) << " " << x.get(GRB_DoubleAttr_X)
	     << endl;
	cout << y.get(GRB_StringAttr_VarName) << " " << y.get(GRB_DoubleAttr_X)
	     << endl;
	cout << z.get(GRB_StringAttr_VarName) << " " << z.get(GRB_DoubleAttr_X)
	     << endl;

	cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    } catch (GRBException e) {
	cout << "Error code = " << e.getErrorCode() << endl;
	cout << e.getMessage() << endl;
    } catch (...) {
	cout << "Exception during optimization" << endl;
    }

	return EXIT_SUCCESS;
} /* ----------  end of function main  ---------- */
