/*
 * pos_euler_con.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cmath>
#include "controller/pos_euler_con.h"

using namespace srcl_ctrl;

PosEulerCon::PosEulerCon()
{
//	kp_0 = 0.75;
//	kd_0 = 1.5;
	kp_0 = 0.95;
	kd_0 = 2.0;
	kp_1 = 0.8;
	kd_1 = 2.0;
	kp_2 = 2.0;
	kd_2 = 1.8;
}

PosEulerCon::~PosEulerCon()
{

}

void PosEulerCon::Update(const QuadState& rs, const PosEulerConInput& input, PosEulerConOutput& output)
{
	float pos_error[3],vel_error[3];

	pos_error[0] = input.pos_d[0] - rs.position_.x;
	pos_error[1] = input.pos_d[1] - rs.position_.y;
	pos_error[2] = input.pos_d[2] - rs.position_.z;

	vel_error[0] = input.vel_d[0] - rs.velocity_.x;
	vel_error[1] = input.vel_d[1] - rs.velocity_.y;
	vel_error[2] = input.vel_d[2] - rs.velocity_.z;

	for(int i = 0; i < 3; i++)
	{
		if(pos_error[i] < 0.001 && pos_error[i] > -0.001)
			pos_error[i] = 0;
		if(vel_error[i] < 0.001 && vel_error[i] > -0.001)
			vel_error[i] = 0;
	}

	float acc_desired[3];

	acc_desired[0] = kp_0 * pos_error[0] + kd_0 * vel_error[0];
	acc_desired[1] = kp_1 * pos_error[1] + kd_1 * vel_error[1];
	acc_desired[2] = kp_2 * pos_error[2] + kd_2 * vel_error[2];

	output.euler_d[0] = 1/rs.g_ * (- acc_desired[1]);
	output.euler_d[1] = 1/rs.g_ * (acc_desired[0]);
	output.euler_d[2] = input.euler_d[2];
	output.delta_w_F = rs.mass_/(8.0*rs.kF_*rs.w_h_) * acc_desired[2];
}
