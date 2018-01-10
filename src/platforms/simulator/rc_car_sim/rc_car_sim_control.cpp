/*
 * rc_car_sim_control.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: rdu
 */

#include <iostream>

#include "rc_car_sim/rc_car_sim_control.h"
#include "utility/logging/logger.hpp"

using namespace librav;

RCCarSimControl::RCCarSimControl()
{
	lcm_ = std::make_shared<lcm::LCM>();

	if (!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	//else {
	//lcm_->subscribe("quad_controller/quad_motion_service", &MotionServer::LcmGoalHandler, &motion_server_);
	//motion_server_ = std::make_shared<MotionServer>(lcm_);

	//data_trans_ = std::make_shared<QuadDataTransmitter>(lcm_);
	//poly_motion_client_ = std::make_shared<PolyMotionClient>(lcm_,"quad_planner/polynomial_curve", "quad_controller/quad_motion_service");
	//}
}

void RCCarSimControl::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
	CtrlLogger &logging_helper = CtrlLogger::GetLogger(log_name_prefix, log_save_path);

	logging_helper.AddItemNameToEntryHead("pos_x");
	logging_helper.AddItemNameToEntryHead("pos_y");
	logging_helper.AddItemNameToEntryHead("pos_z");
	logging_helper.AddItemNameToEntryHead("pos_d_x");
	logging_helper.AddItemNameToEntryHead("pos_d_y");
	logging_helper.AddItemNameToEntryHead("pos_d_z");
	logging_helper.AddItemNameToEntryHead("pos_e_x");
	logging_helper.AddItemNameToEntryHead("pos_e_y");
	logging_helper.AddItemNameToEntryHead("pos_e_z");

	logging_helper.AddItemNameToEntryHead("quat_x");
	logging_helper.AddItemNameToEntryHead("quat_y");
	logging_helper.AddItemNameToEntryHead("quat_z");
	logging_helper.AddItemNameToEntryHead("quat_w");
	logging_helper.AddItemNameToEntryHead("quat_d_x");
	logging_helper.AddItemNameToEntryHead("quat_d_y");
	logging_helper.AddItemNameToEntryHead("quat_d_z");
	logging_helper.AddItemNameToEntryHead("quat_d_w");
	logging_helper.AddItemNameToEntryHead("yaw");
	logging_helper.AddItemNameToEntryHead("yaw_d");

	logging_helper.AddItemNameToEntryHead("vel_x");
	logging_helper.AddItemNameToEntryHead("vel_y");
	logging_helper.AddItemNameToEntryHead("vel_z");
	logging_helper.AddItemNameToEntryHead("vel_d_x");
	logging_helper.AddItemNameToEntryHead("vel_d_y");
	logging_helper.AddItemNameToEntryHead("vel_d_z");
	logging_helper.AddItemNameToEntryHead("vel_e_x");
	logging_helper.AddItemNameToEntryHead("vel_e_y");
	logging_helper.AddItemNameToEntryHead("vel_e_z");

	//	logging_helper.AddItemNameToEntryHead("acc_x");
	//	logging_helper.AddItemNameToEntryHead("acc_y");
	//	logging_helper.AddItemNameToEntryHead("acc_z");
	logging_helper.AddItemNameToEntryHead("acc_d_x");
	logging_helper.AddItemNameToEntryHead("acc_d_y");
	logging_helper.AddItemNameToEntryHead("acc_d_z");

	logging_helper.PassEntryHeaderToLogger();
}

void RCCarSimControl::SetInitPose(float x, float y, float yaw)
{
}

DataToRCCarSim RCCarSimControl::ConvertRobotCmdToSimCmd(const RCCarCmd &cmd)
{
	DataToRCCarSim sim_cmd;

	sim_cmd.cmd.driving_vel_lcmd = cmd.driving_vel_lcmd;
	sim_cmd.cmd.driving_vel_rcmd = cmd.driving_vel_rcmd;
	sim_cmd.cmd.steering_ang_cmd = cmd.steering_ang_cmd;

	return sim_cmd;
}

void RCCarSimControl::UpdateRobotState(const DataFromRCCarSim &data)
{
	RCCarSensorData sensor_data;
	rs_.UpdateRobotState(sensor_data);
}

RCCarCmd RCCarSimControl::UpdateCtrlLoop()
{
	RCCarCmd cmd;

	// process image

	// plan

	// control

	// set control variables
	cmd.driving_vel_lcmd = 50;
	cmd.driving_vel_rcmd = 50;
	cmd.steering_ang_cmd = 45.0f;// * 3.1415f / 180.0f;

	return cmd;
}
