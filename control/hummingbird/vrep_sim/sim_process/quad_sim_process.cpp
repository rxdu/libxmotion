/*
 * car_sim_process.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#include <iostream>
#include <string>

#include <sim_process/quad_sim_process.h>
#include "library/g3log/g3log.hpp"
#include "control/att_euler_con.h"
#include "control/pos_euler_con.h"

#include "control/att_quat_con.h"
#include "control/pos_quat_con.h"

using namespace srcl_ctrl;

QuadSimProcess::QuadSimProcess(int client_id):
	SimProcess(new QuadSimClient(client_id)),
	process_loop_count(0),pos_quat_con(new PosQuatCon(&rs_))
{

}

QuadSimProcess::~QuadSimProcess(void)
{

}

void QuadSimProcess::SimLoopUpdate(void)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(rs_m_);

	//(Extended Kalman Filter)



	/********* update position control *********/
	// invoke position controller update
	// input: (x,y,z,u,v,w)^desired
	// output: (phi, theta, psi, p, q, r)^desired
	ControlInput pos_con_input;
	ControlOutput pos_con_output;
	PosEulerCon pos_con(&rs_);

	double height = 0.3;
	double radius = 0.8;
	double circle_ang_vel = 180.0/180.0*3.14;
	unsigned int time_label1 = 120;
	unsigned int time_label2 = time_label1 + 120;

	ControlOutput att_con_output;

	att_con_output.motor_ang_vel_d[0] = 0;
	att_con_output.motor_ang_vel_d[1] = 0;
	att_con_output.motor_ang_vel_d[2] = 0;
	att_con_output.motor_ang_vel_d[3] = 0;

//	if(process_loop_count < time_label1) {
//		pos_con_input.pos_d[0] = radius;
//		pos_con_input.pos_d[1] = 0.0;
//		pos_con_input.pos_d[2] = height;
//		pos_con_input.euler_d[2] = 0;
//	}
//	else {
//		pos_con_input.pos_d[0] = radius * cos((process_loop_count - 125)*0.01*circle_ang_vel);
//		pos_con_input.pos_d[1] = radius * sin((process_loop_count - 125)*0.01*circle_ang_vel);
//		pos_con_input.pos_d[2] = height;
//		pos_con_input.euler_d[2] = 0;// + (process_loop_count - 125)*0.01*circle_ang_vel;
//	}

//	if(process_loop_count < time_label1) {
//		pos_con_input.pos_d[0] = 0;
//		pos_con_input.pos_d[1] = 0.0;
//		pos_con_input.pos_d[2] = height;
//		pos_con_input.euler_d[2] = 0;
//
//		pos_con_input.vel_d[0] = 0;
//		pos_con_input.vel_d[1] = 0;
//		pos_con_input.vel_d[2] = 0;
//
//		pos_con.Update(&pos_con_input, &pos_con_output);
//
//		ControlInput att_con_input;
//
//		att_con_input.euler_d[0] = pos_con_output.euler_d[0];
//		att_con_input.euler_d[1] = pos_con_output.euler_d[1];
//		att_con_input.euler_d[2] = pos_con_output.euler_d[2];
//
//		//	std::cout << "desired yaw: " << att_con_input.euler_d[2] << std::endl;
//		att_con_input.rot_rate_d[0] = 0;
//		att_con_input.rot_rate_d[1] = 0;
//		att_con_input.rot_rate_d[2] = 0;
//		att_con_input.delta_w_F = pos_con_output.delta_w_F;
//
//		/*------------- test attitude controller -------------*/
//		//	if(process_loop_count >= time_label1){
//		//		att_con_input.euler_d[0] = 0.0/180.0*3.14;
//		//		att_con_input.euler_d[1] = 0.0/180.0*3.14;
//		//		att_con_input.euler_d[2] = 30.0/180.0*3.14;
//		//	}
//		//	att_con_input.delta_w_F = 0;
//		//
//		//	att_con_input.rot_rate_d[0] = 0;
//		//	att_con_input.rot_rate_d[1] = 0;
//		//	att_con_input.rot_rate_d[2] = 0;
//		/*----------------------------------------------------*/
//
//		/********* update attitude control *********/
//		// input: (phi, theta, psi, p, q, r)^desired
//		// output: motor angular velocities
//		AttEulerCon att_con(&rs_);
//		att_con.Update(&att_con_input, &att_con_output);
//	}
//	else{

	if(process_loop_count < time_label1) {
		pos_con_input.pos_d[0] = 0;
		pos_con_input.pos_d[1] = 0;
		pos_con_input.pos_d[2] = height;
		pos_con_input.yaw_d = 0;
//		pos_con_input.pos_d[0] = radius;
//		pos_con_input.pos_d[1] = 0.0;
//		pos_con_input.pos_d[2] = height;
	}
//	else if(process_loop_count < time_label2) {
//		pos_con_input.pos_d[0] = 1.0;
//		pos_con_input.pos_d[1] = 0;
//		pos_con_input.pos_d[2] = 1.0;
//	}
	else
	{
//		pos_con_input.pos_d[0] = 1.0;
//		pos_con_input.pos_d[1] = 1.0;
//		pos_con_input.pos_d[2] = 1.0;
		pos_con_input.pos_d[0] = 0.0;//radius * cos((process_loop_count - 125)*0.01*circle_ang_vel);
		pos_con_input.pos_d[1] = 0.0;//radius * sin((process_loop_count - 125)*0.01*circle_ang_vel);
		pos_con_input.pos_d[2] = height;
		pos_con_input.yaw_d = M_PI;

		if(process_loop_count == time_label1) {
			std::cout << "------------------------------------------------" << std::endl;
			std::cout << "------------------------------------------------" << std::endl;
			std::cout << "------------------------------------------------" << std::endl;
		}
	}


		pos_con_input.vel_d[0] = 0;
		pos_con_input.vel_d[1] = 0;
		pos_con_input.vel_d[2] = 0;

		pos_quat_con->Update(&pos_con_input, &pos_con_output);

		ControlInput quat_con_input;
		//	quat_con_input.quat_d.w() = 1;
		//	quat_con_input.quat_d.x() = 0;
		//	quat_con_input.quat_d.y() = 0;
		//	quat_con_input.quat_d.z() = 0;
		//	if(process_loop_count < time_label1 + 20) {
		//		Eigen::Quaterniond rotd(Eigen::AngleAxisd(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));
		//		quat_con_input.quat_d = rotd;
		//	}
		//	else {
		//		Eigen::Quaterniond rotd(Eigen::AngleAxisd(Eigen::AngleAxisd(M_PI+M_PI/6, Eigen::Vector3d::UnitZ())));
		//		quat_con_input.quat_d = rotd;
		//	}
		Eigen::Quaterniond rotd(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

		//	Eigen::Quaterniond rotx(Eigen::AngleAxisd(M_PI/180.0*5, Eigen::Vector3d::UnitX()));
		//	Eigen::Quaterniond roty(Eigen::AngleAxisd(0, rotx.matrix().col(1)));
		//	Eigen::Quaterniond rotz(Eigen::AngleAxisd(M_PI - M_PI/36, roty.matrix().col(2)));
		//	Eigen::Quaterniond rotd = rotz * roty * rotx;

		quat_con_input.quat_d = pos_con_output.quat_d;
		if(process_loop_count >= time_label1)
			quat_con_input.quat_d = rotd;

//		std::cout << "quaterion desired: "<< quat_con_input.quat_d.w() << " , " << quat_con_input.quat_d.x() << " , " << quat_con_input.quat_d.y() << " , "<<quat_con_input.quat_d.z() << std::endl;
		quat_con_input.ftotal_d = pos_con_output.ftotal_d;
		quat_con_input.rot_rate_d[0] = 0;
		quat_con_input.rot_rate_d[1] = 0;
		quat_con_input.rot_rate_d[2] = 0;
		AttQuatCon attquat_con(&rs_);
		attquat_con.Update(&quat_con_input, &att_con_output);

	// set control variables
	cmd_m_.motor_cmd.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m_.motor_cmd.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m_.motor_cmd.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m_.motor_cmd.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

//	std::cout << "----------------" << std::endl;
//	std::cout<< "desired motor vel: ( "<< cmd_m_.motor_cmd.ang_vel[0] << " , "
//			<< cmd_m_.motor_cmd.ang_vel[1] << " , "
//			<< cmd_m_.motor_cmd.ang_vel[2] << " , "
//			<< cmd_m_.motor_cmd.ang_vel[3] << " ) "<<std::endl;
//	std::cout << "----------------" << std::endl;

	// code below is used for debugging
	process_loop_count++;

#ifdef ENABLE_LOG
	// log data
	/* data format: image(IMG_RES_X * IMG_RES_Y bytes) +							*/
//	if(process_loop_count == 10)
//	{
		std::string str;
		int i,j;

		for(i = 0; i < IMG_RES_Y; i++)
		{
			for(j = 0; j < IMG_RES_X; j++){
//				str += std::to_string((unsigned int)(rs_m.mono_image[i][j]));
				str += std::to_string((unsigned int)(line_det_.bin_image_[i][j]));
				str += ",";
			}
		}


		LOG(INFO) << str;
//	}
#endif
}


