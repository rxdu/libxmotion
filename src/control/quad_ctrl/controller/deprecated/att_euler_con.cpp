/*
 * att_euler_con.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include "controller/att_euler_con.h"

using namespace librav;

AttEulerCon::AttEulerCon()
{
	kp_phi = 135;
	kd_phi = 105;
	kp_theta = 135;
	kd_theta = 105;
	kp_psi = 60;
	kd_psi = 95;
}

AttEulerCon::~AttEulerCon()
{

}

void AttEulerCon::Update(const QuadState& rs, const AttEulerConInput& input, AttEulerConOutput& output)
{
	double euler_error[3];
	double rate_error[3];

	euler_error[0] = input.euler_d[0] - rs.orientation_.x;
	euler_error[1] = input.euler_d[1] - rs.orientation_.y;
	euler_error[2] = input.euler_d[2] - rs.orientation_.z;

	rate_error[0] = input.rot_rate_d[0] - rs.rotation_rate_.x;
	rate_error[1] = input.rot_rate_d[1] - rs.rotation_rate_.y;
	rate_error[2] = input.rot_rate_d[2] - rs.rotation_rate_.z;

	for(int i = 0; i < 3; i++)
	{
		if(euler_error[i] < 0.00001 && euler_error[i] > -0.00001)
			euler_error[i] = 0;
		if(rate_error[i] < 0.00001 && rate_error[i] > -0.00001)
			rate_error[i] = 0;
	}

	double delta_w_phi, delta_w_theta, delta_w_psi;

	delta_w_phi = kp_phi * euler_error[0] + kd_phi * rate_error[0];
	delta_w_theta = kp_theta * euler_error[1] + kd_theta * rate_error[1];
	delta_w_psi = kp_psi * euler_error[2] + kd_psi * rate_error[2];

	double force;
	force = rs.w_h_ + input.delta_w_F;

	output.motor_ang_vel_d[0] = 1 * force + 0 * delta_w_phi + (-1) * delta_w_theta + 1 * delta_w_psi;
	output.motor_ang_vel_d[1] = 1 * force + (-1) * delta_w_phi +   0 * delta_w_theta + (-1) * delta_w_psi;
	output.motor_ang_vel_d[2] = 1 * force + 0 * delta_w_phi + 1 * delta_w_theta + 1 * delta_w_psi;
	output.motor_ang_vel_d[3] = 1 * force + 1 * delta_w_phi + 0 * delta_w_theta + (-1) * delta_w_psi;
}
