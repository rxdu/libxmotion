/*
 * test_corridor.cpp
 *
 *  Created on: Sep 29, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cstdint>
#include <limits>
#include <cstring>
#include <vector>

#include "gurobi_c++.h"

#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_math.h"
#include "polyopt/gurobi_utils.h"

using namespace srcl_ctrl;
using namespace Eigen;

int main(int argc, char* argv[])
{
	uint32_t r_pos = 4;
	uint32_t N_pos = 2 * r_pos - 1;
	uint32_t r_yaw = 2;
	uint32_t N_yaw = 2 * r_yaw - 1;

	uint8_t kf_num = 3;

	MatrixXf keyframe_x_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_y_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_z_vals = MatrixXf::Zero(r_pos, kf_num);
	MatrixXf keyframe_yaw_vals = MatrixXf::Zero(r_yaw, kf_num);
	MatrixXf keyframe_ts = MatrixXf::Zero(1, kf_num);

	std::vector<MatrixXf> keyframe_vals;

	keyframe_x_vals(0,0) = -0.3;
	keyframe_x_vals(0,1) = 0.5;
	keyframe_x_vals(0,2) = 1.05;
	//keyframe_x_vals(0,3) = 0.35;

	keyframe_y_vals(0,0) = 1.15;
	keyframe_y_vals(0,1) = 1.0;
	keyframe_y_vals(0,2) = 1.5;
	//eyframe_y_vals(0,3) = 0.45;

	keyframe_z_vals(0,0) = -0.0;
	keyframe_z_vals(0,1) = 0.15;
	keyframe_z_vals(0,2) = 0.2;
	//keyframe_z_vals(0,3) = 0.15;

	keyframe_yaw_vals(0,0) = 0;
	keyframe_yaw_vals(0,1) = M_PI/18.0;
	keyframe_yaw_vals(0,2) = M_PI/18.0*1.5;
	//keyframe_yaw_vals(0,3) = M_PI/18.0*2.0;

	keyframe_ts(0,0) = 0;
	keyframe_ts(0,1) = 1.2;
	keyframe_ts(0,2) = 3;
	//keyframe_ts(0,3) = 4.5;

	int dim = 2;
	keyframe_vals.push_back(keyframe_x_vals);
	keyframe_vals.push_back(keyframe_y_vals);
	//keyframe_vals.push_back(keyframe_z_vals);

	int nc = 20;
	Eigen::MatrixXf A_cor, b_cor;

	A_cor = MatrixXf::Zero(nc * 2 * dim, (N_pos + 1) * dim * (kf_num - 1));
	b_cor = MatrixXf::Zero(nc * 2 * dim * (kf_num - 1), 1);

	PolyOptMath::GetNonDimCorridorConstrs(N_pos, r_pos, kf_num , nc , 0.05, keyframe_vals, keyframe_ts, A_cor, b_cor);
}


