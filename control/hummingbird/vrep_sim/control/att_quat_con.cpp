/*
 * att_quat_con.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "control/att_quat_con.h"

using namespace srcl_ctrl;

AttQuatCon::AttQuatCon(RobotState* _rs):
	Controller(_rs)
{
	kp_phi = 0.8;
	kd_phi = 0.1;
	kp_theta = 1;
	kd_theta = 0.1;
	kp_psi = 1.2;
	kd_psi = 0.15;
}

AttQuatCon::~AttQuatCon()
{

}

Eigen::Matrix<double,4,1> AttQuatCon::CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure)
{
	Eigen::Matrix<double,4,4> trans;
	Eigen::Matrix<double,4,4> trans_inv;
	Eigen::Matrix<double,4,1> f_motor;
	Eigen::Matrix<double,4,1> ang_vel;

	double d = rs_->arm_length_;
	double c = rs_->kM_/rs_->kF_;

	trans << 1, 1, 1, 1,
			 0,-d, 0, d,
			-d, 0, d, 0,
			 c,-c, c,-c;

//	std::cout<<"tran matrix: \n"<<trans<<std::endl;

	trans_inv = trans.inverse();

//	std::cout<<"desired force/torque: \n"<<force_toqure<<std::endl;
//	std::cout<<"tran inverse: \n"<<trans_inv<<std::endl;
//	std::cout<<"tran inverse element: \n"<<trans_inv(5)<<std::endl;

//	std::cout<<"force elements 0: "<< force_toqure(0) << std::endl;
//	std::cout<<"force elements 1: "<< force_toqure(1) << std::endl;
//	std::cout<<"force elements 2: "<< force_toqure(2) << std::endl;
//	std::cout<<"force elements 3: "<< force_toqure(3) << std::endl;
//	std::cout<<"trans_inv elements 0: "<< trans_inv(0,0) << std::endl;
//	std::cout<<"trans_inv elements 1: "<< trans_inv(1,0) << std::endl;
//	std::cout<<"trans_inv elements 2: "<< trans_inv(2,0) << std::endl;
//	std::cout<<"trans_inv elements 3: "<< trans_inv(3,0) << std::endl;
	f_motor(0) = trans_inv(0,0) * force_toqure(0) + trans_inv(0,1) * force_toqure(1) + trans_inv(0,2) * force_toqure(2) + trans_inv(0,3) * force_toqure(3);
	f_motor(1) = trans_inv(1,0) * force_toqure(0) + trans_inv(1,1) * force_toqure(1) + trans_inv(1,2) * force_toqure(2) + trans_inv(1,3) * force_toqure(3);
	f_motor(2) = trans_inv(2,0) * force_toqure(0) + trans_inv(2,1) * force_toqure(1) + trans_inv(2,2) * force_toqure(2) + trans_inv(2,3) * force_toqure(3);
	f_motor(3) = trans_inv(3,0) * force_toqure(0) + trans_inv(3,1) * force_toqure(1) + trans_inv(3,2) * force_toqure(2) + trans_inv(3,3) * force_toqure(3);

//	std::cout<<"motor force: \n"<<f_motor<<std::endl;

//	f_motor = trans.inverse() * force_toqure;
	ang_vel = f_motor/rs_->kF_;

//	std::cout<<"ang_vel 0: \n"<<ang_vel<<std::endl;

	for(int i = 0; i < 4; i++) {
		if(ang_vel(i) < 0)
			ang_vel(i) = 0;
		ang_vel(i) = std::sqrt(ang_vel(i));
//		ang_vel(i) = std::round(ang_vel(i));
	}

//	std::cout<<"ang_vel 0: \n"<<ang_vel<<std::endl;

//	std::cout<<"motor speed: \n"<<ang_vel<<std::endl;

	return ang_vel;
}

void AttQuatCon::Update(ControlInput *input, ControlOutput *cmd)
{
	Eigen::Quaterniond quat_e;

	quat_e = rs_->quat_.conjugate() * input->quat_d;
	quat_e = quat_e.normalized();

	double qe0 = quat_e.w();
	double M_sign;

	if(qe0 >= 0)
		M_sign = 1;
	else
		M_sign = -1;

	Eigen::Matrix<float,4,1> desired_ft;

	double rate_error[3];
	rate_error[0] = input->rot_rate_d[0] - rs_->rotation_rate_.x;
	rate_error[1] = input->rot_rate_d[1] - rs_->rotation_rate_.y;
	rate_error[2] = input->rot_rate_d[2] - rs_->rotation_rate_.z;

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

	desired_ft(0) = input->ftotal_d; //rs_->kF * (rs_->w_h)*(rs_->w_h) * 4;
	desired_ft(1) = M_sign * kp_phi * quat_e.x() + kd_phi * rate_error[0];
	desired_ft(2) = M_sign * kp_theta * quat_e.y() + kd_theta * rate_error[1];
	desired_ft(3) = M_sign * kp_psi * quat_e.z() + kd_psi * rate_error[2];

	for(int i = 0; i < 4; i++)
		if(desired_ft(i) < 10e-5 && desired_ft(i) > -10e-5)
			desired_ft(i) = 0;

//	std::cout<<"quaternion error: "<< std::setw(13) << quat_e.w()<<" , "<< std::setw(13) <<quat_e.x()<<" , "<< std::setw(13) <<quat_e.y()<<" , "<< std::setw(13) << quat_e.z()
//			<< " , " << std::setw(5) << M_sign
//			<< " , " << std::setw(13) << desired_ft(1)
//			<< " , " << std::setw(13) << desired_ft(2)
//			<< " , " << std::setw(13) << desired_ft(3)<<std::endl;

	std::cout<<"data: "
				<< std::setw(5) << M_sign
				<< " | " << std::setw(12) << quat_e.w()
				<< " , " << std::setw(12) << quat_e.x()
				<< " , " << std::setw(12) << quat_e.y()
				<< " , " << std::setw(12) << quat_e.z()
				<< " | " << std::setw(12) << rate_error[0]
				<< " , " << std::setw(12) << rate_error[1]
				<< " , " << std::setw(12) << rate_error[2]
				<< " | " << std::setw(12) << desired_ft(0)
				<< " , " << std::setw(12) << desired_ft(1)
				<< " , " << std::setw(12) << desired_ft(2)
				<< " , " << std::setw(12) << desired_ft(3)
				<<std::endl;

//	desired_ft(1) = 0;
//	desired_ft(2) = 0;

//	std::cout<<"desired quaternion (w,x,y,z):"<< input->quat_d.w() << " , " << input->quat_d.x() << " , "
//				<<input->quat_d.y() << " , "<< input->quat_d.z() << std::endl;
//	std::cout<<"quaternion error (w,x,y,z):"<< quat_e.w() << " , " << quat_e.x() << " , "
//			<<quat_e.y() << " , "<< quat_e.z() << std::endl;
//	std::cout<<"quaternion error, rate error (z): "<< quat_e.z() << " , " <<rate_error[2]<< std::endl;
//	std::cout << "desired force/toqrue: \n"<< desired_ft << std::endl;

	// calculate desired motor cmd from desired force and torque
	// w1,w2,w3,w4 = 4800,5200,4800,5200
	//	desired_ft << 6.1198,0,0,-0.0120;
	//	force_torque << 5.6310,0,0,0;
	Eigen::Matrix<double,4,1> motor_vel = CalcMotorCmd(desired_ft);

//	std::cout<<"data: "
//			<< std::setw(5) << M_sign
//			<< " | " << std::setw(12) << quat_e.w()
//			<< " , " << std::setw(12) << quat_e.x()
//			<< " , " << std::setw(12) << quat_e.y()
//			<< " , " << std::setw(12) << quat_e.z()
//			<< std::setw(10) << desired_ft(0)
//			<< " , " << std::setw(12) << desired_ft(1)
//			<< " , " << std::setw(12) << desired_ft(2)
//			<< " , " << std::setw(12) << desired_ft(3)
//			<< " * " << std::setw(12) << motor_vel(0)
//			<< " , " << std::setw(12) << motor_vel(1)
//			<< " , " << std::setw(12) << motor_vel(2)
//			<< " , " << std::setw(12) << motor_vel(3)
//			<<std::endl;

	cmd->motor_ang_vel_d[0] = motor_vel(0);
	cmd->motor_ang_vel_d[1] = motor_vel(1);
	cmd->motor_ang_vel_d[2] = motor_vel(2);
	cmd->motor_ang_vel_d[3] = motor_vel(3);
}

