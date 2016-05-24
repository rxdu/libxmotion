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
#include "ctrl_utils/logging/utils_log.h"
#endif

#include "sim_process/quad_sim_process.h"
#include "quad_ctrl/motion_server/trajectory_manager.h"

using namespace srcl_ctrl;

QuadSimProcess::QuadSimProcess(int client_id):
			SimProcess(new QuadSimClient(client_id)),
			process_loop_count(0),
			pos_quat_con_(new PosQuatCon(&rs_)),
			att_quat_con_(new AttQuatCon(&rs_))
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
	ControlInput pos_con_input;
	ControlOutput pos_con_output;

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
		pos_con_input.vel_d[0] = 0;
		pos_con_input.vel_d[1] = 0;
		pos_con_input.vel_d[2] = 0;
		pos_con_input.acc_d[0] = 0;
		pos_con_input.acc_d[1] = 0;
		pos_con_input.acc_d[2] = 0;
		pos_con_input.yaw_d = 0;
	}
	else
	{
		double angle = (process_loop_count - time_label1)*0.01*circle_ang_vel;
		pos_con_input.pos_d[0] = radius * cos(angle - M_PI/2);
		pos_con_input.pos_d[1] = radius * sin(angle - M_PI/2);
		pos_con_input.pos_d[2] = height;

		pos_con_input.vel_d[0] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel;
		pos_con_input.vel_d[1] = radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel;
		pos_con_input.vel_d[2] = 0;

		pos_con_input.acc_d[0] = - radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
		pos_con_input.acc_d[1] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
		pos_con_input.acc_d[2] = 0;

		pos_con_input.yaw_d = angle;
	}

//	pos_con_input.pos_d[0] = 0;
//	pos_con_input.pos_d[1] = 0;
//	pos_con_input.pos_d[2] = height;
//	pos_con_input.vel_d[0] = 0;
//	pos_con_input.vel_d[1] = 0;
//	pos_con_input.vel_d[2] = 0;
//	pos_con_input.acc_d[0] = 0;
//	pos_con_input.acc_d[1] = 0;
//	pos_con_input.acc_d[2] = 0;
//	pos_con_input.yaw_d = 0;

	pos_quat_con_->Update(&pos_con_input, &pos_con_output);

	ControlInput quat_con_input;
	quat_con_input.quat_d = pos_con_output.quat_d;
	quat_con_input.ftotal_d = pos_con_output.ftotal_d;
	quat_con_input.rot_rate_d[0] = 0;
	quat_con_input.rot_rate_d[1] = 0;
	quat_con_input.rot_rate_d[2] = 0;
	att_quat_con_->Update(&quat_con_input, &att_con_output);

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

void QuadSimProcess::SimLoopUpdate(UAVTrajectoryPoint pt)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(rs_m_);

	//(Extended Kalman Filter)

	/********* update position control *********/
	// invoke position controller update
	ControlInput pos_con_input;
	ControlOutput pos_con_output;

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

	if(!pt.point_empty)
	{
		pos_con_input.pos_d[0] = pt.positions[0];
		pos_con_input.pos_d[1] = pt.positions[1];
		pos_con_input.pos_d[2] = pt.positions[2];
		pos_con_input.vel_d[0] = pt.velocities[0];
		pos_con_input.vel_d[1] = pt.velocities[1];
		pos_con_input.vel_d[2] = pt.velocities[2];
		pos_con_input.acc_d[0] = pt.accelerations[0];
		pos_con_input.acc_d[1] = pt.accelerations[1];
		pos_con_input.acc_d[2] = pt.accelerations[2];
		pos_con_input.yaw_d = pt.yaw;
	}
	else
	{
		pos_con_input.pos_d[0] = rs_.position_.x;
		pos_con_input.pos_d[1] = rs_.position_.y;
		pos_con_input.pos_d[2] = rs_.position_.z;
		pos_con_input.vel_d[0] = 0;
		pos_con_input.vel_d[1] = 0;
		pos_con_input.vel_d[2] = 0;
		pos_con_input.acc_d[0] = 0;
		pos_con_input.acc_d[1] = 0;
		pos_con_input.acc_d[2] = 0;
		pos_con_input.yaw_d = 0;
	}

	pos_quat_con_->Update(&pos_con_input, &pos_con_output);

	ControlInput quat_con_input;
	quat_con_input.quat_d = pos_con_output.quat_d;
	quat_con_input.ftotal_d = pos_con_output.ftotal_d;
	quat_con_input.rot_rate_d[0] = 0;
	quat_con_input.rot_rate_d[1] = 0;
	quat_con_input.rot_rate_d[2] = 0;
	att_quat_con_->Update(&quat_con_input, &att_con_output);

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
