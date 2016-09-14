/*
 * quad_polyopt.h
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 *
 *  Description: this configuration tries to find the minimum snap trajectory
 *  	for a quadrotor in the flat output space (x,y,z,yaw).
 */

#ifndef PLANNING_SRC_POLYOPT_QUAD_POLYOPT_H_
#define PLANNING_SRC_POLYOPT_QUAD_POLYOPT_H_

#include <cstdint>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

#include "quad_flat/quad_flattraj.h"
#include "polyopt/traj_optimizer.h"

namespace srcl_ctrl {

class QuadPolyOpt{
public:
	QuadPolyOpt();
	~QuadPolyOpt();

private:
	TrajOptimizer optimizer_;

	const uint32_t r_pos_;
	uint32_t N_pos_;

	const uint32_t r_yaw_;
	uint32_t N_yaw_;

	uint32_t keyframe_num_;

	TrajOptResult traj_[4];

public:
	Eigen::MatrixXf keyframe_x_vals_;
	Eigen::MatrixXf keyframe_y_vals_;
	Eigen::MatrixXf keyframe_z_vals_;
	Eigen::MatrixXf keyframe_yaw_vals_;
	Eigen::MatrixXf keyframe_ts_;

	QuadFlatTraj flat_traj_;

	void SetPositionPolynomialOrder(uint32_t N) { N_pos_ = N; };
	uint32_t GetPositionPolynomialOrder() { return N_pos_; };
	void SetYawPolynomialOrder(uint32_t N) { N_yaw_ = N; };
	uint32_t GetYawPolynomialOrder() { return N_yaw_; };

public:
	void InitOptMatrices(uint32_t keyframe_num);
	void OptimizeFlatTraj();
	void OptimizeFlatTraj(const Eigen::Ref<const Eigen::MatrixXf> keyframe_x_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_y_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_z_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_yaw_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
			uint32_t keyframe_num);
};

}



#endif /* PLANNING_SRC_POLYOPT_QUAD_POLYOPT_H_ */
