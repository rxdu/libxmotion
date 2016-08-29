/*
 * traj_optimizer.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <limits>
#include <ctime>
#include <string>

#include "polyopt/traj_optimizer.h"
#include "polyopt/polyopt_math.h"
#include "polyopt/gurobi_utils.h"

using namespace srcl_ctrl;
using namespace Eigen;

GRBEnv TrajOptimizer::grb_env_ = GRBEnv();

TrajOptimizer::TrajOptimizer():
		r_(4), N_(7),
		kf_num_(2)
{
}

TrajOptimizer::~TrajOptimizer()
{

}

void TrajOptimizer::InitCalcVars()
{
	Q_ = MatrixXf::Zero((kf_num_ - 1) * (N_+1), (kf_num_ - 1) * (N_+1));
	A_eq_ = MatrixXf::Zero((kf_num_ - 1) * 2 * r_, (kf_num_ - 1) * (N_ + 1));
	b_eq_ = MatrixXf::Zero((kf_num_ - 1) * 2 * r_, 1);

	MatrixXf keyframe_vals = MatrixXf::Zero(r_, kf_num_);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num_);
}

void TrajOptimizer::OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order)
{
	kf_num_ = keyframe_num;
	r_ = deriv_order;
	N_ = poly_order;

	if(N_ < 2*r_ - 1)
	{
		N_ = 2*r_ - 1;
		std::cerr << "Inappropriate order is specified for the polynomial to optimize: N < 2*r - 1" << std::endl;
	}

	InitCalcVars();

	try {
		// record start time
		clock_t exec_time;
		exec_time = clock();

		// create a optimization model
		GRBModel model = GRBModel(grb_env_);

		// add variables to model
		std::vector<GRBVar> x;
		uint32_t var_num = (N_ + 1)*(kf_num_ - 1);
		x.resize(var_num);
		for(int i = 0; i < var_num; i++)
		{
			std::string var_name = "sigma"+std::to_string(i);
			x[i] = model.addVar(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, var_name);
		}
		model.update();

		// set objective
		GRBQuadExpr cost_fun;
		PolyOptMath::GetNonDimQMatrices(N_,r_,kf_num_, keyframe_ts, Q_);
		GurobiUtils::GetQuadraticCostFuncExpr(x, Q_, var_num, cost_fun);;
		model.setObjective(cost_fun);

		// add constraints
		PolyOptMath::GetNonDimEqualityConstrs(N_, r_, kf_num_, keyframe_vals, keyframe_ts, A_eq_, b_eq_);
		GurobiUtils::AddLinEqualityConstrExpr(x, A_eq_, b_eq_, var_num, model);

		// optimize model
		model.optimize();

		exec_time = clock() - exec_time;
		std::cout << "Optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

		for(int i = 0; i < var_num; i++)
		{
			if(i!=0 && i%(N_+1) == 0)
				std::cout << std::endl;

			std::cout << x[i].get(GRB_StringAttr_VarName) << " : "
					<< x[i].get(GRB_DoubleAttr_X) << std::endl;
		}

		std::cout << "\nObj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

	} catch(GRBException& e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} catch(...) {
		std::cout << "Exception during optimization" << std::endl;
	}
}

