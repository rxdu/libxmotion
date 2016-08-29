/*
 * quad_polyopt.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include "polyopt/quad_polyopt.h"

using namespace srcl_ctrl;

QuadPolyOpt::QuadPolyOpt():
		r_pos_(4), N_pos_(7),
		r_yaw_(2), N_yaw_(3)
{

}

QuadPolyOpt::~QuadPolyOpt()
{

}

void QuadPolyOpt::OptimizeFlatTraj(const Eigen::Ref<const Eigen::MatrixXf> keyframe_x_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_y_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_z_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_yaw_vals,
			const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
			uint32_t keyframe_num)
{
	std::cout << "optimization called" << std::endl;

	optimizer_.OptimizeTrajectory(keyframe_x_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	optimizer_.OptimizeTrajectory(keyframe_y_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	optimizer_.OptimizeTrajectory(keyframe_z_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	optimizer_.OptimizeTrajectory(keyframe_yaw_vals, keyframe_ts, keyframe_num, N_yaw_, r_yaw_);
}

