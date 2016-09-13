/*
 * quad_sim_controller.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include "quad_sim/quad_sim_controller.h"

using namespace srcl_ctrl;

QuadSimController::QuadSimController():
		pos_quat_con_(new PosQuatCon(&rs_)),
		att_quat_con_(new AttQuatCon(&rs_)),
		send_to_ros_(false)
{
	previous_state_.point_empty = false;
	previous_state_.positions[0] = 0;
	previous_state_.positions[1] = 0;
	previous_state_.positions[2] = 0.5;
	previous_state_.yaw = 0;

	lcm_ = std::make_shared<lcm::LCM>();

	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		lcm_->subscribe("quad_motion_service", &MotionServer::LcmGoalHandler, &motion_server_);

		vis_trans_ = std::make_shared<QuadDataTransmitter>(lcm_);
	}
}

QuadSimController::~QuadSimController()
{
	delete att_quat_con_;
	delete pos_quat_con_;
}

void QuadSimController::SetInitPose(float x, float y, float z, float yaw)
{
	previous_state_.positions[0] = x;
	previous_state_.positions[1] = y;
	previous_state_.positions[2] = z;
	previous_state_.yaw = yaw;
}

QuadDataToSim QuadSimController::ConvertRobotCmdToSimCmd(const QuadCmd& cmd)
{
	QuadDataToSim sim_cmd;

	//std::cout << "quad cmd: ";
	for(int i = 0; i < 4; i++) {
		sim_cmd.motor_cmd.ang_vel[i] = cmd.ang_vel[i];
		//std::cout << sim_cmd.motor_cmd.ang_vel[i] << " , ";
	}
	//std::cout << std::endl;

	return sim_cmd;
}

void QuadSimController::UpdateRobotState(QuadDataFromSim* data)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(*data);

	if(send_to_ros_)
		vis_trans_->SendRobotStateDataToROS(rs_);
}

QuadCmd QuadSimController::UpdateCtrlLoop()
{
	QuadCmd cmd_m;

	UAVTrajectoryPoint pt;
	pt = motion_server_.GetCurrentDesiredPose();

	// if no new point, stay where it was
	if(!pt.point_empty)
	{
		previous_state_ = pt;
	}

	//(Extended Kalman Filter)

	/********* update position control *********/
	// invoke position controller update
	ControlInput pos_con_input;
	ControlOutput pos_con_output;

	pos_con_input.pos_d[0] = previous_state_.positions[0];
	pos_con_input.pos_d[1] = previous_state_.positions[1];
	pos_con_input.pos_d[2] = previous_state_.positions[2];
	pos_con_input.vel_d[0] = previous_state_.velocities[0];
	pos_con_input.vel_d[1] = previous_state_.velocities[1];
	pos_con_input.vel_d[2] = previous_state_.velocities[2];
	pos_con_input.acc_d[0] = previous_state_.accelerations[0];
	pos_con_input.acc_d[1] = previous_state_.accelerations[1];
	pos_con_input.acc_d[2] = previous_state_.accelerations[2];
	pos_con_input.yaw_d = previous_state_.yaw;

	pos_quat_con_->Update(&pos_con_input, &pos_con_output);

	/********* update attitude control *********/
	ControlInput quat_con_input;
	ControlOutput att_con_output;

	att_con_output.motor_ang_vel_d[0] = 0;
	att_con_output.motor_ang_vel_d[1] = 0;
	att_con_output.motor_ang_vel_d[2] = 0;
	att_con_output.motor_ang_vel_d[3] = 0;

	quat_con_input.quat_d = pos_con_output.quat_d;
	quat_con_input.ftotal_d = pos_con_output.ftotal_d;
	quat_con_input.rot_rate_d[0] = 0;
	quat_con_input.rot_rate_d[1] = 0;
	quat_con_input.rot_rate_d[2] = 0;
	att_quat_con_->Update(&quat_con_input, &att_con_output);

	// set control variables
	cmd_m.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

#ifdef ENABLE_LOG
	/* log data */
	UtilsLog::AppendLogMsgTuple4f(cmd_m_.motor_cmd.ang_vel[0],cmd_m_.motor_cmd.ang_vel[1],
			cmd_m_.motor_cmd.ang_vel[2],cmd_m_.motor_cmd.ang_vel[3]);

	// write all data from current iteration into log file
	LOG(INFO) << UtilsLog::GetLogEntry();
	// empty data before a new iteration starts
	UtilsLog::EmptyLogMsgEntry();
#endif

	ctrl_loop_count_++;
	lcm_->handleTimeout(0);

	return cmd_m;
}

QuadCmd QuadSimController::UpdateCtrlLoop(const QuadState& desired)
{
	QuadCmd cmd_m;

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

	if(ctrl_loop_count_ < time_label1) {
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
		double angle = (ctrl_loop_count_ - time_label1)*0.01*circle_ang_vel;
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
	cmd_m.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

#ifdef ENABLE_LOG
	UtilsLog::AppendLogMsgTuple4f(cmd_m_.motor_cmd.ang_vel[0],cmd_m_.motor_cmd.ang_vel[1],
			cmd_m_.motor_cmd.ang_vel[2],cmd_m_.motor_cmd.ang_vel[3]);
#endif

	// code below is used for debugging
	ctrl_loop_count_++;

	return cmd_m;

#ifdef ENABLE_LOG
	/* log data */
	// write all data from current iteration into log file
	LOG(INFO) << UtilsLog::GetLogEntry();
	// empty data before a new iteration starts
	UtilsLog::EmptyLogMsgEntry();
#endif
}


