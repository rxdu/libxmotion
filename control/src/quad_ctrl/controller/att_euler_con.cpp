/*
 * att_euler_con.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include "controller/att_euler_con.h"

using namespace srcl_ctrl;

AttEulerCon::AttEulerCon(RobotState *_rs):
		Controller(_rs)
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

void AttEulerCon::Update(ControlInput *input, ControlOutput *cmd)
{
	double delta_w_phi, delta_w_theta, delta_w_psi;
	double euler_error[3];
	double rate_error[3];

	euler_error[0] = input->euler_d[0] - rs_->orientation_.x;
	euler_error[1] = input->euler_d[1] - rs_->orientation_.y;
	euler_error[2] = input->euler_d[2] - rs_->orientation_.z;

	rate_error[0] = input->rot_rate_d[0] - rs_->rotation_rate_.x;
	rate_error[1] = input->rot_rate_d[1] - rs_->rotation_rate_.y;
	rate_error[2] = input->rot_rate_d[2] - rs_->rotation_rate_.z;

	for(int i = 0; i < 3; i++)
	{
		if(euler_error[i] < 0.00001 && euler_error[i] > -0.00001)
			euler_error[i] = 0;
		if(rate_error[i] < 0.00001 && rate_error[i] > -0.00001)
			rate_error[i] = 0;
	}

	delta_w_phi = kp_phi * euler_error[0] + kd_phi * rate_error[0];
	delta_w_theta = kp_theta * euler_error[1] + kd_theta * rate_error[1];
	delta_w_psi = kp_psi * euler_error[2] + kd_psi * rate_error[2];

//	std::cout<< "euler_d[0]: "<< input->euler_d[0] << " , rs_->orientation.x: "<< rs_->orientation.x <<std::endl;
//	std::cout<< "euler_error[0]: "<< euler_error[0] << " , rate_error[0]: "<< rate_error[0]<<std::endl;
//	std::cout << "euler z desired/actual: "<< input->euler_d[2] << " , " << rs_->orientation.z << std::endl;
//	std::cout << "euler error: "<< euler_error[0] << " , " << euler_error[1] << " , " << euler_error[2] << std::endl;
//	std::cout << "rate error: "<< rate_error[0] << " , " << rate_error[1] << " , " << rate_error[2] << std::endl;
//
//	std::cout<< "(delta_w_phi, delta_w_theta, delta_w_theta):  " << delta_w_phi << " , " << delta_w_theta << " , " << delta_w_psi << std::endl;
//	std::cout<< "delta_w_theta: " << delta_w_theta << std::endl;
//	std::cout<< "delta_w_theta: " << delta_w_psi << std::endl;

	double force;
	force = rs_->w_h_ + input->delta_w_F;

//	std::cout<<"force:"<<force << std::endl;

	cmd->motor_ang_vel_d[0] = 1 * force + 0 * delta_w_phi + (-1) * delta_w_theta + 1 * delta_w_psi;
	cmd->motor_ang_vel_d[1] = 1 * force + (-1) * delta_w_phi +   0 * delta_w_theta + (-1) * delta_w_psi;
	cmd->motor_ang_vel_d[2] = 1 * force + 0 * delta_w_phi + 1 * delta_w_theta + 1 * delta_w_psi;
	cmd->motor_ang_vel_d[3] = 1 * force + 1 * delta_w_phi + 0 * delta_w_theta + (-1) * delta_w_psi;
}
