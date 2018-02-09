/* 
 * rc_tamiya_sim_client.cpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "rc_tamiya_sim/rc_tamiya_sim_client.hpp"

#include <iostream>
#include <cmath>

using namespace librav;

RCTamiyaSimClient::RCTamiyaSimClient() : VrepSimClient<DataFromRCTamiyaSim, DataToRCTamiyaSim>()
{
	// initialize variables
	image_raw_ = new simxUChar[IMG_RES_Y * IMG_RES_X];

	img_res[0] = IMG_RES_X;
	img_res[1] = IMG_RES_Y;

	// initialize communication between server and client
	ConfigDataStreaming();

	std::cout << "INFO: RC car simulation client initialized successfully." << std::endl;
}

RCTamiyaSimClient::~RCTamiyaSimClient()
{
	delete[] image_raw_;
}

void RCTamiyaSimClient::ConfigDataStreaming(void)
{
	// get simulation object handles
	simxGetObjectHandle(client_id_, "tamiya", &car_handle_, simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "main_camera", &camera_handle_, simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "steering_joint_fr", &steering_right_, simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "steering_joint_fl", &steering_left_, simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "driving_joint_front_right", &driving_front_right_, simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_front_left", &driving_front_left_, simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_rear_right", &driving_rear_right_, simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_rear_left", &driving_rear_left_, simx_opmode_oneshot_wait);

	// initialize robot status data streaming
	simxGetObjectFloatParameter(client_id_, driving_rear_right_, 2012, &driving_right_vel_, simx_opmode_streaming);
	simxGetObjectFloatParameter(client_id_, driving_rear_left_, 2012, &driving_left_vel_, simx_opmode_streaming);

	simxGetVisionSensorImage(client_id_, camera_handle_, img_res, &image_raw_, 1, simx_opmode_streaming);

	simxGetObjectVelocity(client_id_, car_handle_, body_lin_vel_, body_ang_vel_, simx_opmode_streaming);
	simxGetJointPosition(client_id_, steering_right_, &steering_angle_, simx_opmode_streaming);
}

bool RCTamiyaSimClient::GetCarDrivingVel(float &rvel, float &lvel, float &body_vel)
{
	// get joint position and joint velocity
	if (simxGetObjectVelocity(client_id_, car_handle_, body_lin_vel_, body_ang_vel_, simx_opmode_buffer) == simx_error_noerror &&
		simxGetObjectFloatParameter(client_id_, driving_rear_right_, 2012, &driving_right_vel_, simx_opmode_buffer) == simx_error_noerror &&
		simxGetObjectFloatParameter(client_id_, driving_rear_left_, 2012, &driving_left_vel_, simx_opmode_buffer) == simx_error_noerror)
	{
		// here you have a valid value!
		body_vel = std::sqrt(body_lin_vel_[0] * body_lin_vel_[0] + body_lin_vel_[1] * body_lin_vel_[1]);
		rvel = driving_rear_right_;
		lvel = driving_rear_left_;

		return true;
	}
	else
		return false;
}

bool RCTamiyaSimClient::GetCarSteeringAngle(float &data)
{
	// get joint position and joint velocity
	if (simxGetJointPosition(client_id_, steering_right_, &steering_angle_, simx_opmode_buffer) == simx_error_noerror)
	{
		// here you have a valid value!
		//		std::cout << "linear: " << "( "<< body_lin_vel_[0] << "," << body_lin_vel_[1] <<","<<body_lin_vel_[2] << " ) ";
		//		std::cout << "angular: " << "( "<< body_ang_vel_[0] << "," << body_ang_vel_[1] <<","<<body_ang_vel_[2] << " )" << std::endl;

		data = steering_angle_;

		return true;
	}
	else
		return false;
}

bool RCTamiyaSimClient::GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X])
{
	if (simxGetVisionSensorImage(client_id_, camera_handle_, img_res, &image_raw_, 1, simx_opmode_buffer) == simx_error_noerror)
	{
		// here you have a valid value!
		int i, j;

		for (i = 0; i < IMG_RES_Y; i++)
			for (j = 0; j < IMG_RES_X; j++)
				img[i][j] = image_raw_[(IMG_RES_Y - i - 1) * IMG_RES_X + j];

		return true;
	}
	else
		return false;
}

void RCTamiyaSimClient::SetCarDrivingVel(float cmd)
{
	// add limits to speed
	if (cmd > max_forward_speed)
		cmd = max_forward_speed;
	if (cmd < max_reverse_speed)
		cmd = max_reverse_speed;

	// front wheel driving 
	simxSetJointTargetVelocity(client_id_, driving_front_right_, cmd, simx_opmode_oneshot);
	simxSetJointTargetVelocity(client_id_, driving_front_left_, cmd, simx_opmode_oneshot);

	// rear wheel driving
	simxSetJointTargetVelocity(client_id_, driving_rear_right_, -cmd, simx_opmode_oneshot);
	simxSetJointTargetVelocity(client_id_, driving_rear_left_, -cmd, simx_opmode_oneshot);
}

void RCTamiyaSimClient::SetCarSteeringAngle(float cmd)
{
	double cmd_radian = cmd / 180.0 * M_PI;

	if (cmd_radian > max_steer_angle)
		cmd_radian = max_steer_angle;
	if (cmd_radian < -max_steer_angle)
		cmd_radian = -max_steer_angle;

	simxSetJointTargetPosition(client_id_, steering_right_, cmd_radian, simx_opmode_oneshot);
	simxSetJointTargetPosition(client_id_, steering_left_, cmd_radian, simx_opmode_oneshot);
}

void RCTamiyaSimClient::SetCarSteeringVelocity(float cmd)
{
	simxSetJointTargetVelocity(client_id_, steering_right_, cmd, simx_opmode_oneshot);
	simxSetJointTargetVelocity(client_id_, steering_left_, cmd, simx_opmode_oneshot);
}

bool RCTamiyaSimClient::ReceiveDataFromSimRobot(DataFromRCTamiyaSim *rdata)
{
	if (GetCarDrivingVel(rdata->driving_vel_right, rdata->driving_vel_left, rdata->body_vel) &&
		GetCarSteeringAngle(rdata->steering_ang) &&
		GetVisionImage(rdata->mono_image))
		return true;
	else
		return false;
}

void RCTamiyaSimClient::UpdateCtrlLoop(const DataFromRCTamiyaSim &rdata, DataToRCTamiyaSim *rcmd)
{
	rcmd->driving_vel_cmd = 5;
	rcmd->steering_ang_cmd = -30;
}

void RCTamiyaSimClient::SendDataToSimRobot(const DataToRCTamiyaSim &rcmd)
{
	SetCarDrivingVel(rcmd.driving_vel_cmd);
	SetCarSteeringAngle(rcmd.steering_ang_cmd);
}
