/*
 * quad_solo_sim_controller.cpp
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#include <iostream>
#include "quad_solo_sim/quad_solo_sim_controller.h"
#include "common/logging_helper.h"

using namespace srcl_ctrl;

QuadSoloSimController::QuadSoloSimController():
		att_con_(new AttQuatCon(rs_)),
		pos_con_(new PosQuatCon(rs_)),
		broadcast_rs_(false)
{
	rs_.arm_length_ = 0.205;
	rs_.mass_ = 1.2 + 0.02465 * 4;
	rs_.w_h_ = sqrt(rs_.mass_ * rs_.g_ / 4 / rs_.kF_);
	att_con_->UpdateQuadParams();

	att_con_->SetControlGains(1.0, 0.1, 1.0, 0.1, 1.2, 0.15);
	pos_con_->SetControlGains(3.8, 0.08, 3.2, 3.8, 0.08, 3.2, 1.8, 0.05, 1.85);

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

QuadSoloSimController::~QuadSoloSimController()
{
}

// This function must be called before entering the control loop.
void  QuadSoloSimController::InitLogger(std::string log_name_prefix, std::string log_save_path)
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
#endif
}

void QuadSoloSimController::SetInitPose(float x, float y, float z, float yaw)
{
	previous_state_.positions[0] = x;
	previous_state_.positions[1] = y;
	previous_state_.positions[2] = z;
	previous_state_.yaw = yaw;
}

DataToQuadSim QuadSoloSimController::ConvertRobotCmdToSimCmd(const QuadCmd& cmd)
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

void QuadSoloSimController::UpdateRobotState(const DataFromQuadSim& data)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(data);

	if(broadcast_rs_)
		data_trans_->SendQuadStateData(rs_);
}

QuadCmd QuadSoloSimController::UpdateCtrlLoop()
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

	LoggingHelper::GetInstance().AddItemDataToEntry("quat_x", rs_.quat_.x());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_y", rs_.quat_.y());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_z", rs_.quat_.z());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_w", rs_.quat_.w());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_d_x", att_con_input.quat_d.x());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_d_y", att_con_input.quat_d.y());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_d_z", att_con_input.quat_d.z());
	LoggingHelper::GetInstance().AddItemDataToEntry("quat_d_w", att_con_input.quat_d.w());

	LoggingHelper::GetInstance().AddItemDataToEntry("force", att_con_input.ftotal_d);

	// write all data from current iteration into log file
	LoggingHelper::GetInstance().PassEntryDataToLogger();
#endif

	ctrl_loop_count_++;
	lcm_->handleTimeout(0);

	return cmd_m;
}
