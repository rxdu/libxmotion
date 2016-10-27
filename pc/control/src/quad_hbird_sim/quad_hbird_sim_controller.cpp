/*
 * quad_sim_controller.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#include <iostream>

#include "quad_hbird_sim/quad_hbird_sim_controller.h"
#ifdef ENABLE_G3LOG
#include "ctrl_utils/logging/logging_helper.h"
#endif

using namespace srcl_ctrl;

QuadHbirdSimController::QuadHbirdSimController():
		pos_quat_con_(new PosQuatCon(rs_)),
		att_quat_con_(new AttQuatCon(rs_)),
		broadcast_rs_(false)
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
		lcm_->subscribe("quad_controller/quad_motion_service", &MotionServer::LcmGoalHandler, &motion_server_);

		data_trans_ = std::make_shared<QuadDataTransmitter>(lcm_);
	}
}

QuadHbirdSimController::~QuadHbirdSimController()
{
}

void  QuadHbirdSimController::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
#ifdef ENABLE_G3LOG
	LoggingHelper& logging_helper = LoggingHelper::GetInstance(log_name_prefix, log_save_path);

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

//	logging_helper.AddItemNameToEntryHead("acc_x");
//	logging_helper.AddItemNameToEntryHead("acc_y");
//	logging_helper.AddItemNameToEntryHead("acc_z");
	logging_helper.AddItemNameToEntryHead("acc_d_x");
	logging_helper.AddItemNameToEntryHead("acc_d_y");
	logging_helper.AddItemNameToEntryHead("acc_d_z");

	logging_helper.AddItemNameToEntryHead("jerk_d_x");
	logging_helper.AddItemNameToEntryHead("jerk_d_y");
	logging_helper.AddItemNameToEntryHead("jerk_d_z");

	logging_helper.AddItemNameToEntryHead("omega_d_x");
	logging_helper.AddItemNameToEntryHead("omega_d_y");
	logging_helper.AddItemNameToEntryHead("omega_d_z");

	logging_helper.PassEntryHeaderToLogger();
#endif
}

void QuadHbirdSimController::SetInitPose(float x, float y, float z, float yaw)
{
	previous_state_.positions[0] = x;
	previous_state_.positions[1] = y;
	previous_state_.positions[2] = z;
	previous_state_.yaw = yaw;
}

DataToQuadSim QuadHbirdSimController::ConvertRobotCmdToSimCmd(const QuadCmd& cmd)
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

void QuadHbirdSimController::UpdateRobotState(const DataFromQuadSim& data)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(data);

	if(broadcast_rs_)
		data_trans_->SendQuadStateData(rs_);
}

QuadCmd QuadHbirdSimController::UpdateCtrlLoop()
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
	pos_con_input.jerk_d[0] = previous_state_.jerks[0];
	pos_con_input.jerk_d[1] = previous_state_.jerks[1];
	pos_con_input.jerk_d[2] = previous_state_.jerks[2];
	pos_con_input.yaw_d = previous_state_.yaw;
	pos_con_input.yaw_rate_d = previous_state_.yaw_rate;

	pos_quat_con_->Update(pos_con_input, pos_con_output);

	/********* update attitude control *********/
	AttQuatConInput quat_con_input;
	AttQuatConOutput att_con_output;

	att_con_output.motor_ang_vel_d[0] = 0;
	att_con_output.motor_ang_vel_d[1] = 0;
	att_con_output.motor_ang_vel_d[2] = 0;
	att_con_output.motor_ang_vel_d[3] = 0;

	quat_con_input.quat_d = pos_con_output.quat_d;
	quat_con_input.ftotal_d = pos_con_output.ftotal_d;
//	quat_con_input.rot_rate_d[0] = 0;
//	quat_con_input.rot_rate_d[1] = 0;
//	quat_con_input.rot_rate_d[2] = 0;
	quat_con_input.rot_rate_d[0] = pos_con_output.rot_rate_d[0];
	quat_con_input.rot_rate_d[1] = pos_con_output.rot_rate_d[1];
	quat_con_input.rot_rate_d[2] = pos_con_output.rot_rate_d[2];
	att_quat_con_->Update(quat_con_input, att_con_output);

	// set control variables
	cmd_m.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

	//std::cout << "pos x desired: " << previous_state_.positions[0] << std::endl;

#ifdef ENABLE_G3LOG
	/* log data */
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_x", rs_.position_.x);
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_y", rs_.position_.y);
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_z", rs_.position_.z);
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_d_x", previous_state_.positions[0]);
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_d_y", previous_state_.positions[1]);
	LoggingHelper::GetInstance().AddItemDataToEntry("pos_d_z", previous_state_.positions[2]);

	LoggingHelper::GetInstance().AddItemDataToEntry("vel_x", rs_.velocity_.x);
	LoggingHelper::GetInstance().AddItemDataToEntry("vel_y", rs_.velocity_.y);
	LoggingHelper::GetInstance().AddItemDataToEntry("vel_z", rs_.velocity_.z);
	LoggingHelper::GetInstance().AddItemDataToEntry("vel_d_x", previous_state_.velocities[0]);
	LoggingHelper::GetInstance().AddItemDataToEntry("vel_d_y", previous_state_.velocities[1]);
	LoggingHelper::GetInstance().AddItemDataToEntry("vel_d_z", previous_state_.velocities[2]);

//	LoggingHelper::GetInstance().AddItemDataToEntry("acc_x", rs_.);
//	LoggingHelper::GetInstance().AddItemDataToEntry("acc_y", rs_.velocity_.y);
//	LoggingHelper::GetInstance().AddItemDataToEntry("acc_z", rs_.velocity_.z);
	LoggingHelper::GetInstance().AddItemDataToEntry("acc_d_x", previous_state_.accelerations[0]);
	LoggingHelper::GetInstance().AddItemDataToEntry("acc_d_y", previous_state_.accelerations[1]);
	LoggingHelper::GetInstance().AddItemDataToEntry("acc_d_z", previous_state_.accelerations[2]);

	LoggingHelper::GetInstance().AddItemDataToEntry("jerk_d_x", previous_state_.jerks[0]);
	LoggingHelper::GetInstance().AddItemDataToEntry("jerk_d_y", previous_state_.jerks[1]);
	LoggingHelper::GetInstance().AddItemDataToEntry("jerk_d_z", previous_state_.jerks[2]);

	// write all data from current iteration into log file
	LoggingHelper::GetInstance().PassEntryDataToLogger();
#endif

	ctrl_loop_count_++;
	lcm_->handleTimeout(0);

	return cmd_m;
}
