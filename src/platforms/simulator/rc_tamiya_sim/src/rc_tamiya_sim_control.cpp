/* 
 * rc_car_sim_control.cpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "rc_tamiya_sim/rc_tamiya_sim_control.hpp"

#include <iostream>

#include "logging/logger.hpp"

using namespace librav;

RCTamiyaSimControl::RCTamiyaSimControl()
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

void RCTamiyaSimControl::InitLogger(std::string log_name_prefix, std::string log_save_path)
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

	logging_helper.PassEntryHeaderToLogger();
}

void RCTamiyaSimControl::SetInitPose(float x, float y, float yaw)
{
}

void RCTamiyaSimControl::UpdateRobotState(const DataFromRCTamiyaSim &data)
{
	// RCCarSensorData sensor_data;
	// rs_.UpdateRobotState(sensor_data);
}

DataToRCTamiyaSim RCTamiyaSimControl::UpdateCtrlLoop()
{
	DataToRCTamiyaSim cmd;

	// process image

	// plan

	// control

	// set control variables
	cmd.driving_vel_cmd = 50;
	cmd.steering_ang_cmd = 45.0f;// * 3.1415f / 180.0f;

	return cmd;
}
