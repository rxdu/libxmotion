/*
 * att_quat_con.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#include <iostream>
#include <iomanip>
#include <cmath>

#ifdef ENABLE_LOG
#include "g3log/g3log.hpp"
#endif

#include "quad_ctrl/controller/att_quat_con.h"

using namespace srcl_ctrl;

AttQuatCon::AttQuatCon(const QuadState& _rs):
		rs_(_rs)
{
//	kp_phi = 1;
//	kd_phi = 0.1;
//	kp_theta = 1;
//	kd_theta = 0.1;
	kp_phi = 1.25;
	kd_phi = 0.06;
	kp_theta = 1.25;
	kd_theta = 0.06;
//	kp_psi = 1.2;
//	kd_psi = 0.15;
	// 0.08, 0.05
	kp_psi = 0.08;
	kd_psi = 0.03;

	double d = rs_.arm_length_;
	double c = rs_.kM_/rs_.kF_;

	plus_type_trans_ << 1, 1, 1, 1,
			 0,-d, 0, d,
			-d, 0, d, 0,
			 c,-c, c,-c;
	plus_type_trans_inv_ = plus_type_trans_.inverse();

	x_type_trans_ << 1, 1, 1, 1,
			std::sqrt(2.0)*d/2.0,-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			c,-c, c,-c;
	x_type_trans_inv_ = x_type_trans_.inverse();
}

AttQuatCon::~AttQuatCon()
{

}

void AttQuatCon::UpdateQuadParams()
{
	double d = rs_.arm_length_;
	double c = rs_.kM_/rs_.kF_;

	plus_type_trans_ << 1, 1, 1, 1,
			0,-d, 0, d,
			-d, 0, d, 0,
			c,-c, c,-c;
	plus_type_trans_inv_ = plus_type_trans_.inverse();

	x_type_trans_ << 1, 1, 1, 1,
			std::sqrt(2.0)*d/2.0,-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			-std::sqrt(2.0)*d/2.0, -std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0, std::sqrt(2.0)*d/2.0,
			c,-c, c,-c;
	x_type_trans_inv_ = x_type_trans_.inverse();
}

Eigen::Matrix<double,4,1> AttQuatCon::CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type)
{
	Eigen::Matrix<double,4,4> trans_inv;
	Eigen::Matrix<double,4,1> f_motor;
	Eigen::Matrix<double,4,1> ang_vel;

	if(type == QuadFlightType::PLUS_TYPE) {
		trans_inv = plus_type_trans_inv_;
	}
	else if(type == QuadFlightType::X_TYPE) {
		trans_inv = x_type_trans_inv_;
	}

	f_motor(0) = trans_inv(0,0) * force_toqure(0) + trans_inv(0,1) * force_toqure(1) + trans_inv(0,2) * force_toqure(2) + trans_inv(0,3) * force_toqure(3);
	f_motor(1) = trans_inv(1,0) * force_toqure(0) + trans_inv(1,1) * force_toqure(1) + trans_inv(1,2) * force_toqure(2) + trans_inv(1,3) * force_toqure(3);
	f_motor(2) = trans_inv(2,0) * force_toqure(0) + trans_inv(2,1) * force_toqure(1) + trans_inv(2,2) * force_toqure(2) + trans_inv(2,3) * force_toqure(3);
	f_motor(3) = trans_inv(3,0) * force_toqure(0) + trans_inv(3,1) * force_toqure(1) + trans_inv(3,2) * force_toqure(2) + trans_inv(3,3) * force_toqure(3);

	ang_vel = f_motor/rs_.kF_;

	for(int i = 0; i < 4; i++) {
		if(ang_vel(i) < 0)
			ang_vel(i) = 0;
		ang_vel(i) = std::sqrt(ang_vel(i));
	}

	return ang_vel;
}

void AttQuatCon::Update(const AttQuatConInput& input, AttQuatConOutput& output)
{
	Eigen::Quaterniond quat_e;

	//qe = q - qd
	quat_e = rs_.quat_.conjugate() * input.quat_d;
	quat_e = quat_e.normalized();

	double qe0 = quat_e.w();
	double M_sign;

	if(qe0 >= 0)
		M_sign = 1;
	else
		M_sign = -1;

	Eigen::Matrix<float,4,1> desired_ft;

	double rate_error[3];
	rate_error[0] = input.rot_rate_d[0] - rs_.rotation_rate_.x;
	rate_error[1] = input.rot_rate_d[1] - rs_.rotation_rate_.y;
	rate_error[2] = input.rot_rate_d[2] - rs_.rotation_rate_.z;

	for(int i = 0; i < 4; i++){
		if(rate_error[i] < 10e-6 && rate_error[i] > -10e-6)
			rate_error[i] = 0;
	}

	if(quat_e.x() < 10e-6 && quat_e.x() > -10e-6)
		quat_e.x() = 0;
	if(quat_e.y() < 10e-6 && quat_e.y() > -10e-6)
		quat_e.y() = 0;
	if(quat_e.z() < 10e-6 && quat_e.z() > -10e-6)
		quat_e.z() = 0;

	desired_ft(0) = input.ftotal_d; //rs_->kF * (rs_->w_h)*(rs_->w_h) * 4;
	desired_ft(1) = M_sign * kp_phi * quat_e.x() + kd_phi * rate_error[0];
	desired_ft(2) = M_sign * kp_theta * quat_e.y() + kd_theta * rate_error[1];
	desired_ft(3) = M_sign * kp_psi * quat_e.z() + kd_psi * rate_error[2];

	for(int i = 0; i < 4; i++)
		if(desired_ft(i) < 10e-5 && desired_ft(i) > -10e-5)
			desired_ft(i) = 0;

	// calculate desired motor cmd from desired force and torque
	Eigen::Matrix<double,4,1> motor_vel = CalcMotorCmd(desired_ft, rs_.quad_flight_type_);

	output.motor_ang_vel_d[0] = motor_vel(0);
	output.motor_ang_vel_d[1] = motor_vel(1);
	output.motor_ang_vel_d[2] = motor_vel(2);
	output.motor_ang_vel_d[3] = motor_vel(3);
}
