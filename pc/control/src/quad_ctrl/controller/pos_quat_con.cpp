/*
 * pos_quat_con.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#ifdef ENABLE_LOG
#include "g3log/g3log.hpp"
#endif

#include "quad_ctrl/controller/pos_quat_con.h"

using namespace srcl_ctrl;

PosQuatCon::PosQuatCon(const QuadState& _rs):
		rs_(_rs),
		zint_uppper_limit(0.1),zint_lower_limit(-1.0),
		xyint_uppper_limit(0.8), xyint_lower_limit(-0.8)
{
	// 0-1: 3.8, 0.08, 3.2
	kp_0 = 3.8;
	ki_0 = 0.08;
	kd_0 = 3.2;

	kp_1 = 3.8;
	ki_1 = 0.08;
	kd_1 = 3.2;

	// kp kd 1.8 2.45
//	kp_2 = 1.2;
//	ki_2 = 0.08;
//	kd_2 = 1.85;
	// 1.25, 0.145, 1.65
	kp_2 = 1.8;
	ki_2 = 0.05;
	kd_2 = 1.85;

	pos_e_integral[0] = 0.0;
	pos_e_integral[1] = 0.0;
	pos_e_integral[2] = 0.0;
}

PosQuatCon::~PosQuatCon()
{

}

void PosQuatCon::Update(const PosQuatConInput& input, PosQuatConOutput& output)
{
	float pos_error[3],vel_error[3];

	pos_error[0] = input.pos_d[0] - rs_.position_.x;
	pos_error[1] = input.pos_d[1] - rs_.position_.y;
	pos_error[2] = input.pos_d[2] - rs_.position_.z;

	vel_error[0] = input.vel_d[0] - rs_.velocity_.x;
	vel_error[1] = input.vel_d[1] - rs_.velocity_.y;
	vel_error[2] = input.vel_d[2] - rs_.velocity_.z;

	for(int i = 0; i < 3; i++)
	{
		if(pos_error[i] < 0.002 && pos_error[i] > -0.002)
			pos_error[i] = 0;
		if(vel_error[i] < 0.002 && vel_error[i] > -0.002)
			vel_error[i] = 0;
	}

	pos_e_integral[0] = pos_e_integral[0] + pos_error[0];
	pos_e_integral[1] = pos_e_integral[1] + pos_error[1];
	pos_e_integral[2] = pos_e_integral[2] + pos_error[2];

	float acc_desired[3];

	acc_desired[0] = kp_0 * pos_error[0] + ki_0 * pos_e_integral[0] + kd_0 * vel_error[0];
	acc_desired[1] = kp_1 * pos_error[1] + ki_1 * pos_e_integral[1] + kd_1 * vel_error[1];
	acc_desired[2] = kp_2 * pos_error[2] + ki_2 * pos_e_integral[2] + kd_2 * vel_error[2];

	if(pos_e_integral[0] > xyint_uppper_limit)
		pos_e_integral[0] = xyint_uppper_limit;
	if(pos_e_integral[0] < xyint_lower_limit)
		pos_e_integral[0] = xyint_lower_limit;

	if(pos_e_integral[1] > xyint_uppper_limit)
		pos_e_integral[1] = xyint_uppper_limit;
	if(pos_e_integral[1] < xyint_lower_limit)
		pos_e_integral[1] = xyint_lower_limit;

	if(pos_e_integral[2] > zint_uppper_limit)
		pos_e_integral[2] = zint_uppper_limit;
	if(pos_e_integral[2] < zint_lower_limit)
		pos_e_integral[2] = zint_lower_limit;

	Eigen::Vector3d Fi(acc_desired[0]+input.acc_d[0], acc_desired[1]+input.acc_d[1], acc_desired[2] + input.acc_d[2] + rs_.g_);
	//	Eigen::Vector3d Fi(acc_desired[0], acc_desired[1], acc_desired[2] + rs_->g_);
	Eigen::Vector3d Fi_n;
	Eigen::Vector3d Fb_n(0,0,1);

	Fi_n = Fi.normalized();

	Eigen::Vector4d qd_n;
	Eigen::Vector3d FbFi_cross;
	double FbT_Fi;
	double scale;
	double qd_wd;
	Eigen::Quaterniond quat_pr;

	FbT_Fi = Fb_n.transpose() * Fi_n;

	if(FbT_Fi < 0)
	{
		Fb_n(2) = -1;
		FbT_Fi = Fb_n.transpose() * Fi_n;
	}

	qd_wd = 1+FbT_Fi;
	scale = 1/sqrt(2*qd_wd);
	FbFi_cross = Fb_n.cross(Fi_n);

	quat_pr.w() = qd_wd/scale;
	quat_pr.x() = FbFi_cross(0)/scale;
	quat_pr.y() = FbFi_cross(1)/scale;
	quat_pr.z() = FbFi_cross(2)/scale;

//	quat_y.w() = cos(input->yaw_d/2);
//	quat_y.x() = 0;
//	quat_y.y() = 0;
//	quat_y.z() = sin(input->yaw_d/2);

	quat_pr.normalize();
	Eigen::Quaterniond quat_y(Eigen::AngleAxisd(input.yaw_d, quat_pr.matrix().col(2)));
	Eigen::Quaterniond quat_result = quat_y * quat_pr;

	output.quat_d = quat_result.normalized();
	output.ftotal_d = Fi.norm() * rs_.mass_;

	if(output.quat_d.x() < 10e-6 && output.quat_d.x() > -10e-6)
		output.quat_d.x() = 0;
	if(output.quat_d.y() < 10e-6 && output.quat_d.y() > -10e-6)
		output.quat_d.y() = 0;
	if(output.quat_d.z() < 10e-6 && output.quat_d.z() > -10e-6)
		output.quat_d.z() = 0;
	if(output.quat_d.w() < 10e-6 && output.quat_d.w() > -10e-6)
		output.quat_d.w() = 0;
}
