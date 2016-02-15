/*
 * quad_sim_client.cpp
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <vrep_client/quad_sim_client.h>
#include <iostream>
#include <cmath>

#include "library/g3log/g3log.hpp"

using namespace srcl_ctrl;

#define SIM_TARGET_VEL	9999

QuadSimClient::QuadSimClient(simxInt clientId):
		RobotSimClient(clientId)
{
	// initialize variables
	image_raw_ = new simxUChar[IMG_RES_Y * IMG_RES_X];

	img_res[0] = IMG_RES_X;
	img_res[1] = IMG_RES_Y;

	quad_pos[0] = 0;
	gyro_sig = NULL;
	gyro_sig_size = 0;
	acc_sig = NULL;
	acc_sig_size = 0;

	// get simulation object handles
	simxGetObjectHandle(client_id_, "asctec_hummingbird",&quad_handle_,simx_opmode_oneshot_wait);


	// initialize communication between server and client
	ConfigDataStreaming();

	std::cout << "INFO: Quadrotor simulation client initialized successfully." << std::endl;
}

QuadSimClient::~QuadSimClient()
{
	delete[] image_raw_;
}

void QuadSimClient::ConfigDataStreaming(void)
{
	// initialize robot status data streaming
	simxGetObjectPosition(client_id_, quad_handle_, -1, quad_pos,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"imu_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"imu_acc",&acc_sig,&acc_sig_size,simx_opmode_streaming);

	// initialize motor values
	simxSetFloatSignal(client_id_, "propeller_cmd_front", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_right", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_left", 0.0, simx_opmode_oneshot);
}

//bool QuadSimClient::GetCarSteeringAngle(float *data)
//{
//	// get joint position and joint velocity
//	if(simxGetJointPosition(client_id_,steering_right_, &steering_angle_ ,simx_opmode_buffer) == simx_error_noerror)
//	{
//		// here you have a valid value!
//		//		std::cout << "linear: " << "( "<< body_lin_vel_[0] << "," << body_lin_vel_[1] <<","<<body_lin_vel_[2] << " ) ";
//		//		std::cout << "angular: " << "( "<< body_ang_vel_[0] << "," << body_ang_vel_[1] <<","<<body_ang_vel_[2] << " )" << std::endl;
//
//		*data = steering_angle_;
//
//		return true;
//	}
//	else
//		return false;
//}

bool QuadSimClient::GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X])
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

bool QuadSimClient::ReceiveGyroData(IMU_DataType *data)
{
	if (simxGetStringSignal(client_id_,"hummingbird_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=gyro_sig_size/4;

		if(cnt == 3)
		{
			data->raw_x=((float*)gyro_sig)[0];
			data->raw_y=((float*)gyro_sig)[1];
			data->raw_z=((float*)gyro_sig)[2];
		}

		return true;

//		std::cout << "gyro (x, y, z) = " << "( " << gyro_data[0] <<" , " << gyro_data[1] << " , " << gyro_data[2] << " )" << std::endl;
	}
	else
		return false;
}

bool QuadSimClient::ReceiveAccData(IMU_DataType *data)
{
	if (simxGetStringSignal(client_id_,"hummingbird_acc",&acc_sig,&acc_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=acc_sig_size/4;

		if(cnt == 3)
		{
			data->raw_x=((float*)acc_sig)[0];
			data->raw_y=((float*)acc_sig)[1];
			data->raw_z=((float*)acc_sig)[2];
		}

		return true;
//		std::cout << "acc (x, y, z) = " << "( " << acc_data[0] <<" , " << acc_data[1] << " , " << acc_data[2] << " )" << std::endl;
	}
	else
		return false;
}

bool QuadSimClient::ReceiveQuadPosition(void)
{
	if (simxGetObjectPosition(client_id_, quad_handle_, -1, quad_pos,simx_opmode_buffer) == simx_error_noerror)
	{
//		std::cout << "pos (x, y, z) = " << "( " << pos[0] <<" , " << pos[1] << " , " << pos[2] << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadSimClient::ReceiveQuadPosition(Position *data)
{
	if (simxGetObjectPosition(client_id_, quad_handle_, -1, quad_pos,simx_opmode_buffer) == simx_error_noerror)
	{
//		std::cout << "pos (x, y, z) = " << "( " << pos[0] <<" , " << pos[1] << " , " << pos[2] << " )" << std::endl;
		(*data).x = quad_pos[0];
		(*data).y = quad_pos[1];
		(*data).z = quad_pos[2];
		return true;
	}
	else
		return false;
}

void QuadSimClient::SendPropellerCmd(QuadCmd cmd)
{
	float front_prop, rear_prop, left_prop, right_prop;

	front_prop = cmd.ang_vel[0];
	rear_prop = cmd.ang_vel[1];
	left_prop = cmd.ang_vel[2];
	right_prop = cmd.ang_vel[3];

	// add limits

	// send commands to simulator
	simxSetFloatSignal(client_id_, "propeller_cmd_front", front_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear", rear_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_left", left_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_right", right_prop, simx_opmode_oneshot);

//	if(motor_update_result != 0)
//		std::cout << "ERROR: Failed to update one or more motor commands." << std::endl;
}

bool QuadSimClient::ReceiveDataFromRobot(DataFromRobot *rx_data)
{
	if(ReceiveGyroData(&(rx_data->imu_data.gyro)) &&
			ReceiveAccData(&(rx_data->imu_data.acc)) &&
			ReceiveQuadPosition(&(rx_data->pos)))
		return true;
	else
		return false;
}

void QuadSimClient::SendDataToRobot(const DataToRobot &rcmd)
{
	SendPropellerCmd(rcmd.vel_cmd);
}
