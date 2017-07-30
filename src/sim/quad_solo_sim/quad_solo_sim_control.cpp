/*
 * quad_solo_sim_control.cpp
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#include "quad_solo_sim/quad_solo_sim_control.h"

#include <iostream>

#include "utility/logging/logger.h"

using namespace librav;

QuadSoloSimControl::QuadSoloSimControl():
		att_con_(new AttQuatCon(rs_)),
		pos_con_(new PosQuatCon(rs_)),
		broadcast_rs_(false)
{
	rs_.arm_length_ = 0.205;
	rs_.mass_ = 1.2 + 0.02465 * 4;
	rs_.w_h_ = sqrt(rs_.mass_ * rs_.g_ / 4 / rs_.kF_);

	AttQuatCon::ParamType att_param;
	att_param.kp_phi = 1.0;
	att_param.kd_phi = 0.1;
	att_param.kp_theta = 1.0;
	att_param.kd_theta = 0.1;
	att_param.kp_psi = 1.2;
	att_param.kd_psi = 0.15;
	att_con_->InitParams(att_param);

	PosQuatCon::ParamType pos_param;
	pos_param.kp_0 = 3.8;
	pos_param.ki_0 = 0.08;
	pos_param.kd_0 = 3.2;
	pos_param.kp_1 = 3.8;
	pos_param.ki_1 = 0.08;
	pos_param.kd_1 = 3.2;
	pos_param.kp_2 = 1.8;
	pos_param.ki_2 = 0.05;
	pos_param.kd_2 = 1.85;

	pos_param.zint_uppper_limit = 0.1;
	pos_param.zint_lower_limit = -1.0;
	pos_param.xyint_uppper_limit = 0.8;
	pos_param.xyint_lower_limit = -0.8;

	pos_param.ts_ = 0.01;
	pos_con_->InitParams(pos_param);

	previous_state_.point_empty = false;
	previous_state_.positions[0] = 0;
	previous_state_.positions[1] = 0;
	previous_state_.positions[2] = 0.5;
	previous_state_.yaw = 0;

	lcm_ = std::make_shared<lcm::LCM>();

	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		//lcm_->subscribe("quad_controller/quad_motion_service", &MotionServer::LcmGoalHandler, &motion_server_);
		motion_server_ = std::make_shared<MotionServer>(lcm_);
		data_trans_ = std::make_shared<QuadDataTransmitter>(lcm_);
	}
}

// This function must be called before entering the control loop.
void  QuadSoloSimControl::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
	CtrlLogger& logging_helper = CtrlLogger::InitLogger(log_name_prefix, log_save_path);

	logging_helper.AddItemNameToEntryHead("pos_x");
	logging_helper.AddItemNameToEntryHead("pos_y");
	logging_helper.AddItemNameToEntryHead("pos_z");
	logging_helper.AddItemNameToEntryHead("pos_d_x");
	logging_helper.AddItemNameToEntryHead("pos_d_y");
	logging_helper.AddItemNameToEntryHead("pos_d_z");

	logging_helper.AddItemNameToEntryHead("vel_x");
	logging_helper.AddItemNameToEntryHead("vel_y");
	logging_helper.AddItemNameToEntryHead("vel_z");
	logging_helper.AddItemNameToEntryHead("vel_d_x");
	logging_helper.AddItemNameToEntryHead("vel_d_y");
	logging_helper.AddItemNameToEntryHead("vel_d_z");

	logging_helper.AddItemNameToEntryHead("quat_x");
	logging_helper.AddItemNameToEntryHead("quat_y");
	logging_helper.AddItemNameToEntryHead("quat_z");
	logging_helper.AddItemNameToEntryHead("quat_w");
	logging_helper.AddItemNameToEntryHead("quat_d_x");
	logging_helper.AddItemNameToEntryHead("quat_d_y");
	logging_helper.AddItemNameToEntryHead("quat_d_z");
	logging_helper.AddItemNameToEntryHead("quat_d_w");

	logging_helper.AddItemNameToEntryHead("force");

	logging_helper.PassEntryHeaderToLogger();
}

void QuadSoloSimControl::SetInitPose(float x, float y, float z, float yaw)
{
	previous_state_.positions[0] = x;
	previous_state_.positions[1] = y;
	previous_state_.positions[2] = z;
	previous_state_.yaw = yaw;
}

DataToQuadSim QuadSoloSimControl::ConvertRobotCmdToSimCmd(const QuadCmd& cmd)
{
	DataToQuadSim sim_cmd;

	//std::cout << "quad cmd: ";
	for(int i = 0; i < 4; i++) {
		sim_cmd.ang_vel[i] = cmd.ang_vel[i];
		//std::cout << sim_cmd.motor_cmd.ang_vel[i] << " , ";
	}
	//std::cout << std::endl;

	return sim_cmd;
}

void QuadSoloSimControl::UpdateRobotState(const DataFromQuadSim& data)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(data);

	if(broadcast_rs_)
		data_trans_->SendQuadStateData(rs_);
}

QuadCmd QuadSoloSimControl::UpdateCtrlLoop()
{
	// this sim runs at 100 Hz, so system time increase at a step of 10 ms
	data_trans_->SendSystemTime(ctrl_loop_count_*10);

	QuadCmd cmd_m;

	UAVTrajectoryPoint pt;
	pt = motion_server_->GetCurrentDesiredState(ctrl_loop_count_ *10);

	// if no new point, stay where it was
	if(!pt.point_empty)
	{
		previous_state_ = pt;
	}

	//(Extended Kalman Filter)

	/********* update position control *********/
	// invoke position controller update
	PosQuatConInput pos_con_input;
	PosQuatConOutput pos_con_output;

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

	pos_con_->Update(pos_con_input, pos_con_output);

	/********* update attitude control *********/
	AttQuatConInput att_con_input;
	AttQuatConOutput att_con_output;

	att_con_input.quat_d = pos_con_output.quat_d;
	att_con_input.ftotal_d = pos_con_output.ftotal_d;
	att_con_input.rot_rate_d[0] = 0;
	att_con_input.rot_rate_d[1] = 0;
	att_con_input.rot_rate_d[2] = 0;
	att_con_->Update(att_con_input, att_con_output);

	// set control variables
	cmd_m.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

	/* log data */
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_x", rs_.position_.x);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_y", rs_.position_.y);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_z", rs_.position_.z);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_d_x", previous_state_.positions[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_d_y", previous_state_.positions[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("pos_d_z", previous_state_.positions[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("vel_x", rs_.velocity_.x);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_y", rs_.velocity_.y);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_z", rs_.velocity_.z);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_d_x", previous_state_.velocities[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_d_y", previous_state_.velocities[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("vel_d_z", previous_state_.velocities[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("quat_x", rs_.quat_.x());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_y", rs_.quat_.y());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_z", rs_.quat_.z());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_w", rs_.quat_.w());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_x", att_con_input.quat_d.x());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_y", att_con_input.quat_d.y());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_z", att_con_input.quat_d.z());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_w", att_con_input.quat_d.w());

	CtrlLogger::GetLogger().AddItemDataToEntry("force", att_con_input.ftotal_d);

	// write all data from current iteration into log file
	CtrlLogger::GetLogger().PassEntryDataToLogger();

	ctrl_loop_count_++;
	lcm_->handleTimeout(0);

	return cmd_m;
}
