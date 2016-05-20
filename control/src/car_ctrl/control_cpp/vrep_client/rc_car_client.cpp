/*
 * armsim_client.cpp
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <vrep_client/rc_car_client.h>
#include <iostream>
#include <cmath>

#include "library/g3log/g3log.hpp"

using namespace RobotToolkitRIVeR;

#define SIM_TARGET_VEL	9999

RCCarClient::RCCarClient(simxInt clientId):
		RobotSimClient(clientId)
{
	// initialize variables
	image_raw_ = new simxUChar[IMG_RES_Y * IMG_RES_X];

	img_res[0] = IMG_RES_X;
	img_res[1] = IMG_RES_Y;

	// get simulation object handles
	simxGetObjectHandle(client_id_, "rc_car",&car_handle_,simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "main_camera",&camera_handle_,simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "steering_joint_fr",&steering_right_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "steering_joint_fl",&steering_left_,simx_opmode_oneshot_wait);

	simxGetObjectHandle(client_id_, "driving_joint_front_right",&driving_front_right_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_front_left",&driving_front_left_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_rear_right",&driving_rear_right_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "driving_joint_rear_left",&driving_rear_left_,simx_opmode_oneshot_wait);

	// initialize communication between server and client
	ConfigDataStreaming();

	std::cout << "INFO: RC car simulation client initialized successfully." << std::endl;
}

RCCarClient::~RCCarClient()
{
	delete[] image_raw_;
}

void RCCarClient::ConfigDataStreaming(void)
{
	// initialize robot status data streaming
//	simxGetJointPosition(client_id_,joint1_handle_,&q_[0],simx_opmode_streaming);
//	simxGetJointPosition(client_id_,joint2_handle_,&q_[1],simx_opmode_streaming);

	simxGetObjectFloatParameter(client_id_,driving_rear_right_,2012,&driving_right_vel_,simx_opmode_streaming);
	simxGetObjectFloatParameter(client_id_,driving_rear_left_,2012,&driving_left_vel_,simx_opmode_streaming);

	simxGetVisionSensorImage(client_id_,camera_handle_,img_res, &image_raw_, 1, simx_opmode_streaming);

	simxGetObjectVelocity(client_id_, car_handle_, body_lin_vel_, body_ang_vel_, simx_opmode_streaming);
	simxGetJointPosition(client_id_,steering_right_, &steering_angle_ ,simx_opmode_streaming);
}

bool RCCarClient::GetCarDrivingVel(float *rvel, float *lvel, float *body_vel)
{
	// get joint position and joint velocity
	if(simxGetObjectVelocity(client_id_,car_handle_,body_lin_vel_, body_ang_vel_,simx_opmode_buffer) == simx_error_noerror &&
			simxGetObjectFloatParameter(client_id_,driving_rear_right_,2012,&driving_right_vel_,simx_opmode_buffer) == simx_error_noerror &&
			simxGetObjectFloatParameter(client_id_,driving_rear_right_,2012,&driving_right_vel_,simx_opmode_buffer) == simx_error_noerror )
	{
		// here you have a valid value!
		*body_vel = std::sqrt(body_lin_vel_[0] * body_lin_vel_[0] + body_lin_vel_[1] * body_lin_vel_[1]);
		*rvel = driving_rear_right_;
		*lvel = driving_rear_left_;

		return true;
	}
	else
		return false;
}

bool RCCarClient::GetCarSteeringAngle(float *data)
{
	// get joint position and joint velocity
	if(simxGetJointPosition(client_id_,steering_right_, &steering_angle_ ,simx_opmode_buffer) == simx_error_noerror)
	{
		// here you have a valid value!
		//		std::cout << "linear: " << "( "<< body_lin_vel_[0] << "," << body_lin_vel_[1] <<","<<body_lin_vel_[2] << " ) ";
		//		std::cout << "angular: " << "( "<< body_ang_vel_[0] << "," << body_ang_vel_[1] <<","<<body_ang_vel_[2] << " )" << std::endl;

		*data = steering_angle_;

		return true;
	}
	else
		return false;
}

bool RCCarClient::GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X])
{
	if(simxGetVisionSensorImage(client_id_,camera_handle_,img_res, &image_raw_, 1, simx_opmode_buffer) == simx_error_noerror)
	{
		// here you have a valid value!
		int i,j;

		for(i = 0; i < IMG_RES_Y; i++)
			for(j = 0; j < IMG_RES_X; j++)
				img[i][j] = image_raw_[(IMG_RES_Y - i - 1)*IMG_RES_X+j];

		return true;
	}
	else
		return false;
}

void RCCarClient::SetCarDrivingVel(const float &rcmd, const float &lcmd)
{
	float vel_right;
	float vel_left;

	// internal conversion for simulation
	vel_right = -rcmd;
	vel_left = -lcmd;

	// add limits to speed
	if(vel_right >= 60)
		vel_right = 60;
	if(vel_right <= -60)
		vel_right = -60;
	if(vel_left >= 60)
		vel_left = 60;
	if(vel_left <= -60)
		vel_left = -60;

	// front wheel driving is disabled
//	simxSetJointTargetVelocity(client_id_,driving_front_right_,cmd,simx_opmode_oneshot);
//	simxSetJointTargetVelocity(client_id_,driving_front_left_,cmd,simx_opmode_oneshot);

	// rear wheel driving is enabled
	simxSetJointTargetVelocity(client_id_,driving_rear_right_,vel_right,simx_opmode_oneshot);
	simxSetJointTargetVelocity(client_id_,driving_rear_left_,vel_left,simx_opmode_oneshot);
}

void RCCarClient::SetCarSteeringAngle(const float &cmd)
{
	simxSetJointTargetPosition(client_id_,steering_right_,-cmd,simx_opmode_oneshot);
	simxSetJointTargetPosition(client_id_,steering_left_,-cmd,simx_opmode_oneshot);
}

bool RCCarClient::ReceiveDataFromRobot(DataFromRobot *rx_data)
{
	if(GetCarDrivingVel(&(rx_data->vs.driving_vel_right), &(rx_data->vs.driving_vel_left), &(rx_data->vs.body_vel)) &&
			GetCarSteeringAngle(&(rx_data->vs.steering_ang)) &&
			GetVisionImage(rx_data->mono_image))
		return true;
	else
		return false;
}

void RCCarClient::SendDataToRobot(const DataToRobot &rcmd)
{
	SetCarDrivingVel(rcmd.cmd.driving_vel_rcmd,rcmd.cmd.driving_vel_lcmd);
	SetCarSteeringAngle(rcmd.cmd.steering_ang_cmd);
}
