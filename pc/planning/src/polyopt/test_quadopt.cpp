/*
 * test_quadopt.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"

#include "polyopt/quad_polyopt.h"

using namespace srcl_ctrl;
using namespace Eigen;

int main(int argc, char* argv[])
{
	uint32_t r_pos = 4;
	uint32_t N_pos = 2 * r_pos - 1;
	uint32_t r_yaw = 2;
	uint32_t N_yaw = 2 * r_yaw - 1;

	uint8_t kf_num = 4;

	MatrixXf keyframe_x_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_y_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_z_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_yaw_vals = MatrixXf::Zero(r_yaw, kf_num);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num);

	keyframe_x_vals(0,0) = -0.15;
	keyframe_x_vals(0,1) = 0.25;
	keyframe_x_vals(0,2) = 0.3;
	keyframe_x_vals(0,3) = 0.35;

//	keyframe_x_vals(1,0) = 0;
//	keyframe_x_vals(1,1) = std::numeric_limits<float>::infinity();
//	keyframe_x_vals(1,2) = std::numeric_limits<float>::infinity();
//	keyframe_x_vals(1,3) = 0;

	keyframe_y_vals(0,0) = -0.2;
	keyframe_y_vals(0,1) = 0.3;
	keyframe_y_vals(0,2) = 0.35;
	keyframe_y_vals(0,3) = 0.45;

	keyframe_z_vals(0,0) = -0.0;
	keyframe_z_vals(0,1) = 0.15;
	keyframe_z_vals(0,2) = 0.2;
	keyframe_z_vals(0,3) = 0.15;

	keyframe_yaw_vals(0,0) = 0;
	keyframe_yaw_vals(0,1) = M_PI/18.0;
	keyframe_yaw_vals(0,2) = M_PI/18.0*1.5;
	keyframe_yaw_vals(0,3) = M_PI/18.0*2.0;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;
	keyframe_ts(0,2) = 3;
	keyframe_ts(0,3) = 4.5;

	QuadPolyOpt opt;
	opt.OptimizeFlatTraj(keyframe_x_vals, keyframe_y_vals, keyframe_z_vals, keyframe_yaw_vals, keyframe_ts, kf_num);
	opt.flat_traj_.print();
}

