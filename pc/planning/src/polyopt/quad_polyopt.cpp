/*
 * quad_polyopt.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <ctime>
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

	clock_t exec_time;
	exec_time = clock();

	traj_[0] = optimizer_.OptimizeTrajectory(keyframe_x_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[1] = optimizer_.OptimizeTrajectory(keyframe_y_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[2] = optimizer_.OptimizeTrajectory(keyframe_z_vals, keyframe_ts, keyframe_num, N_pos_, r_pos_);
	std::cout << "\n-------------------------------------------------------------------------\n" << std::endl;
	traj_[3] = optimizer_.OptimizeTrajectory(keyframe_yaw_vals, keyframe_ts, keyframe_num, N_yaw_, r_yaw_);

	exec_time = clock() - exec_time;
	std::cout << "All optimization finished in " << double(exec_time)/CLOCKS_PER_SEC << " s.\n" << std::endl;

	for(int i = 0; i < traj_[0].segments.size(); i++)
	{
		std::vector<std::vector<double>> segcoeffs;
		double ts,te;
		for(int j = 0; j < 4; j++)
			segcoeffs.push_back(traj_[j].segments[i].coeffs);

		ts = traj_[0].segments[i].ts;
		te = traj_[0].segments[i].te;
		flat_traj_.AddTrajSeg(segcoeffs, ts, te);
	}
}

