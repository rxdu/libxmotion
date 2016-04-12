/*
 * car_sim_process.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#include <iostream>
#include <string>

#ifdef ENABLE_LOG
#include "g3log/g3log.hpp"
#endif

#include "sim_process/quad_sim_process.h"
#include "controller/att_euler_con.h"
#include "controller/pos_euler_con.h"
#include "controller/att_quat_con.h"
#include "controller/pos_quat_con.h"

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

	double height = 0.5;
	double radius = 1.5;
	double circle_ang_vel = 180.0/180.0*3.14;
	unsigned int time_label1 = 120;

	ControlOutput att_con_output;

	att_con_output.motor_ang_vel_d[0] = 0;
	att_con_output.motor_ang_vel_d[1] = 0;
	att_con_output.motor_ang_vel_d[2] = 0;
	att_con_output.motor_ang_vel_d[3] = 0;

	/********* update attitude control *********/

	if(process_loop_count < time_label1) {
		pos_con_input.pos_d[0] = 0;
		pos_con_input.pos_d[1] = -radius;
		pos_con_input.pos_d[2] = height;
		pos_con_input.yaw_d = 0;
	}
	else
	{
		double angle = (process_loop_count - time_label1)*0.01*circle_ang_vel;
		pos_con_input.pos_d[0] = radius * cos(angle - M_PI/2);
		pos_con_input.pos_d[1] = radius * sin(angle - M_PI/2);
		pos_con_input.pos_d[2] = height;
		pos_con_input.yaw_d = angle; //0;//M_PI/2;
//		pos_con_input.pos_d[0] = 0;
//		pos_con_input.pos_d[1] = 0;
//		pos_con_input.pos_d[2] = height;
//		pos_con_input.yaw_d = M_PI/2;// - M_PI * 15.0/180.0 ;//M_PI;
	}

	pos_con_input.vel_d[0] = 0;
	pos_con_input.vel_d[1] = 0;
	pos_con_input.vel_d[2] = 0;

	pos_quat_con->Update(&pos_con_input, &pos_con_output);

	ControlInput quat_con_input;

	Eigen::Quaterniond rotd(Eigen::AngleAxisd(M_PI*90.0/180.0, Eigen::Vector3d::UnitZ()));

//	Eigen::Quaterniond rotx(Eigen::AngleAxisd(M_PI*15.0/180.0, Eigen::Vector3d::UnitX()));
//	Eigen::Quaterniond roty(Eigen::AngleAxisd(M_PI*15.0/180.0, rotx.matrix().col(1)));
//	Eigen::Quaterniond rotz(Eigen::AngleAxisd(M_PI*15.0/180.0, roty.matrix().col(2)));
//	Eigen::Quaterniond rotd = rotz * roty * rotx;

	quat_con_input.quat_d = pos_con_output.quat_d;
//	if(process_loop_count >= time_label1)
//		quat_con_input.quat_d = rotd;
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

#ifdef ENABLE_LOG
	UtilsLog::AppendLogMsgTuple4f(cmd_m_.motor_cmd.ang_vel[0],cmd_m_.motor_cmd.ang_vel[1],
			cmd_m_.motor_cmd.ang_vel[2],cmd_m_.motor_cmd.ang_vel[3]);
#endif

	// code below is used for debugging
	process_loop_count++;

#ifdef ENABLE_LOG
	/* log data */
	// write all data from current iteration into log file
	LOG(INFO) << UtilsLog::GetLogEntry();
	// empty data before a new iteration starts
	UtilsLog::EmptyLogMsgEntry();
#endif
}
