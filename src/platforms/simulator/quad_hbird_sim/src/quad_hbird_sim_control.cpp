/* 
 * quad_hbird_sim_control.cpp
 * 
 * Created on: Sep 2, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <iostream>

#include "quad_hbird_sim/quad_hbird_sim_control.hpp"
#include "logging/loggers.hpp"

using namespace librav;

QuadHbirdSimControl::QuadHbirdSimControl():
		pos_quat_con_(new PosQuatCon(rs_)),
		att_quat_con_(new AttQuatCon(rs_)),
		mixer_(new QuadMixer(rs_)),
		broadcast_rs_(false)
{
	AttQuatCon::ParamType att_param;
	att_param.kp_phi = 1.25;
	att_param.kd_phi = 0.06;
	att_param.kp_theta = 1.25;
	att_param.kd_theta = 0.06;
	att_param.kp_psi = 0.08;
	att_param.kd_psi = 0.03;
	att_quat_con_->InitParams(att_param);

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
	pos_quat_con_->InitParams(pos_param);

	rs_.quad_flight_type_ = QuadFlightType::X_TYPE;

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

		data_trans_ = std::make_shared<QuadDataBroadcaster>(lcm_);
		//poly_motion_client_ = std::make_shared<PolyMotionClient>(lcm_,"quad_planner/polynomial_curve", "quad_controller/quad_motion_service");
	}
}

void  QuadHbirdSimControl::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
	CtrlLogger& logging_helper = CtrlLogger::GetLogger(log_name_prefix, GetLogFolderPath()+log_save_path);

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

	logging_helper.AddItemNameToEntryHead("jerk_d_x");
	logging_helper.AddItemNameToEntryHead("jerk_d_y");
	logging_helper.AddItemNameToEntryHead("jerk_d_z");

	logging_helper.AddItemNameToEntryHead("omega_d_x");
	logging_helper.AddItemNameToEntryHead("omega_d_y");
	logging_helper.AddItemNameToEntryHead("omega_d_z");

	logging_helper.PassEntryHeaderToLogger();
}

void QuadHbirdSimControl::SetInitPose(float x, float y, float z, float yaw)
{
	previous_state_.positions[0] = x;
	previous_state_.positions[1] = y;
	previous_state_.positions[2] = z;
	previous_state_.yaw = yaw;
}

void QuadHbirdSimControl::UpdateRobotState(const DataFromQuadSim& data)
{
	/********* update robot state *********/
	// Test without state estimator
	QuadSensorData sensor_data;

	// get values directly from simulator, will be replaced by state estimator later
	sensor_data.pos_i.x = data.pos_i.x;
	sensor_data.pos_i.y = data.pos_i.y;
	sensor_data.pos_i.z = data.pos_i.z;

	sensor_data.vel_i.x = data.vel_i.x;
	sensor_data.vel_i.y = data.vel_i.y;
	sensor_data.vel_i.z = data.vel_i.z;

	// euler in X-Y-Z convension
	sensor_data.rot_i.x = data.rot_i.x;
	sensor_data.rot_i.y = data.rot_i.y;
	sensor_data.rot_i.z = data.rot_i.z;

	// quaternion
	sensor_data.quat_i.x = data.quat_i.x;
	sensor_data.quat_i.y = data.quat_i.y;
	sensor_data.quat_i.z = data.quat_i.z;
	sensor_data.quat_i.w = data.quat_i.w;

	sensor_data.rot_rate_b.x = data.rot_rate_b.x;
	sensor_data.rot_rate_b.y = data.rot_rate_b.y;
	sensor_data.rot_rate_b.z = data.rot_rate_b.z;

	sensor_data.laser_points = data.laser_points;

	rs_.UpdateRobotState(sensor_data);

	if(broadcast_rs_)
		data_trans_->SendQuadStateData(rs_);
}

DataToQuadSim QuadHbirdSimControl::UpdateCtrlLoop()
{
	// send system time first
	// this sim runs at 100 Hz, so system time increase at a step of 10 ms
	data_trans_->SendSystemTime(ctrl_loop_count_*10);

	// start doing control work
	DataToQuadSim cmd_m;

	UAVTrajectoryPoint pt;
	pt.point_empty = true;
//	pt = motion_server_->GetCurrentDesiredPose();
	pt = motion_server_->GetCurrentDesiredState(ctrl_loop_count_ *10);
	//pt.point_empty = true;
	motion_server_->ReportProgress();

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

	pos_quat_con_->Update(pos_con_input, &pos_con_output);

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
	att_quat_con_->Update(quat_con_input, &att_con_output);

	// set control variables
	// cmd_m.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	// cmd_m.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	// cmd_m.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	// cmd_m.ang_vel[3] = att_con_output.motor_ang_vel_d[3];
	auto cmd_val = mixer_->CalcMotorCmd(pos_con_output.ftotal_d, att_con_output.torque_d, rs_.quad_flight_type_);
	cmd_m.ang_vel[0] = cmd_val.ang_vel[0];
	cmd_m.ang_vel[1] = cmd_val.ang_vel[1];
	cmd_m.ang_vel[2] = cmd_val.ang_vel[2];
	cmd_m.ang_vel[3] = cmd_val.ang_vel[3];


	//std::cout << "pos x desired: " << previous_state_.positions[0] << std::endl;

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

//	ControlLogger::GetLogger().AddItemDataToEntry("acc_x", rs_.);
//	ControlLogger::GetLogger().AddItemDataToEntry("acc_y", rs_.velocity_.y);
//	ControlLogger::GetLogger().AddItemDataToEntry("acc_z", rs_.velocity_.z);
	CtrlLogger::GetLogger().AddItemDataToEntry("acc_d_x", previous_state_.accelerations[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("acc_d_y", previous_state_.accelerations[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("acc_d_z", previous_state_.accelerations[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("jerk_d_x", previous_state_.jerks[0]);
	CtrlLogger::GetLogger().AddItemDataToEntry("jerk_d_y", previous_state_.jerks[1]);
	CtrlLogger::GetLogger().AddItemDataToEntry("jerk_d_z", previous_state_.jerks[2]);

	CtrlLogger::GetLogger().AddItemDataToEntry("yaw", rs_.orientation_.z);
	CtrlLogger::GetLogger().AddItemDataToEntry("yaw_d", previous_state_.yaw);
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_x", rs_.quat_.x());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_y", rs_.quat_.y());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_z", rs_.quat_.z());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_w", rs_.quat_.w());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_x", quat_con_input.quat_d.x());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_y", quat_con_input.quat_d.y());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_z", quat_con_input.quat_d.z());
	CtrlLogger::GetLogger().AddItemDataToEntry("quat_d_w", quat_con_input.quat_d.w());

	// write all data from current iteration into log file
	CtrlLogger::GetLogger().PassEntryDataToLogger();

	ctrl_loop_count_++;
	lcm_->handleTimeout(0);

	return cmd_m;
}
