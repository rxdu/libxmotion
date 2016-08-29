/*
 * traj_optimizer.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 *
 *  Description: this configuration tries to minimize r'th derivative of
 *  	a N-order polynomial.
 */

#ifndef PLANNING_SRC_POLYOPT_TRAJ_OPTIMIZER_H_
#define PLANNING_SRC_POLYOPT_TRAJ_OPTIMIZER_H_

#include <cstdint>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

namespace srcl_ctrl {

class TrajOptimizer {
public:
	TrajOptimizer();
	~TrajOptimizer();

private:
	// only one environment needed
	static GRBEnv grb_env_;

	// derivative to optimize
	uint32_t r_;
	// highest order of polynomial
	uint32_t N_;
	// number of key frames
	uint32_t kf_num_;

	Eigen::MatrixXf Q_;
	Eigen::MatrixXf A_eq_;
	Eigen::MatrixXf b_eq_;

private:
	void InitCalcVars();

public:
	void OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order);
};

}



#endif /* PLANNING_SRC_POLYOPT_TRAJ_OPTIMIZER_H_ */
