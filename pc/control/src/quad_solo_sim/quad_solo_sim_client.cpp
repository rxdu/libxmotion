/*
 * quad_solo_sim_client.cpp
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#include <iostream>

#include "quad_solo_sim/quad_solo_sim_client.h"

using namespace srcl_ctrl;

QuadSoloSimClient::QuadSoloSimClient():
				RobotSimClient<DataFromQuadSim, DataToQuadSim>(),
				max_motor_speed_(10000)
				{
	// initialize variables
	image_raw_ = new simxUChar[IMG_RES_Y * IMG_RES_X];

	img_res[0] = IMG_RES_X;
	img_res[1] = IMG_RES_Y;

	gyro_sig = nullptr;
	gyro_sig_size = 0;
	acc_sig = nullptr;
	acc_sig_size = 0;
	quat_sig = nullptr;
	quat_sig_size = 0;

	for(int i = 0; i < 3; i++)
	{
		quad_pos[i] = 0;
		quad_linear_vel[i] = 0;
		quad_angular_vel[i] = 0;
		quad_ori[i] = 0;
	}

	// initialize communication between server and client
	ConfigDataStreaming();

	std::cout << "INFO: Quadrotor simulation client initialized successfully." << std::endl;
				}

QuadSoloSimClient::QuadSoloSimClient(simxInt clientId):
				RobotSimClient<DataFromQuadSim, DataToQuadSim>(clientId),
				max_motor_speed_(10000)
				{
	// initialize variables
	image_raw_ = new simxUChar[IMG_RES_Y * IMG_RES_X];

	img_res[0] = IMG_RES_X;
	img_res[1] = IMG_RES_Y;

	gyro_sig = nullptr;
	gyro_sig_size = 0;
	acc_sig = nullptr;
	acc_sig_size = 0;
	quat_sig = nullptr;
	quat_sig_size = 0;

	for(int i = 0; i < 3; i++)
	{
		quad_pos[i] = 0;
		quad_linear_vel[i] = 0;
		quad_angular_vel[i] = 0;
		quad_ori[i] = 0;
	}

	// initialize communication between server and client
	ConfigDataStreaming();

	std::cout << "INFO: Quadrotor simulation client initialized successfully." << std::endl;
				}

QuadSoloSimClient::~QuadSoloSimClient()
{
	delete[] image_raw_;
	delete gyro_sig;
	delete acc_sig;
}

void QuadSoloSimClient::ConfigDataStreaming(void)
{
	// get simulation object handles
	simxGetObjectHandle(client_id_, "quad_3dr_solo",&quad_handle_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "ctrl_ref",&ref_handle_,simx_opmode_oneshot_wait);

	// initialize robot status data streaming
	simxGetObjectPosition(client_id_, ref_handle_, -1, quad_pos,simx_opmode_streaming);
	simxGetObjectVelocity(client_id_, ref_handle_, quad_linear_vel, quad_angular_vel,simx_opmode_streaming);
	simxGetObjectOrientation(client_id_, ref_handle_, -1, quad_ori, simx_opmode_streaming);

	simxGetStringSignal(client_id_,"solo_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"solo_quat",&quat_sig,&quat_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"solo_acc",&acc_sig,&acc_sig_size,simx_opmode_streaming);
	//simxGetStringSignal(client_id_,"3d_scanner_points",&scannerptr_sig,&scannerptr_sig_size,simx_opmode_streaming);

	// initialize motor values
	simxSetFloatSignal(client_id_, "propeller_cmd_front_left", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear_right", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_front_right", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear_left", 0.0, simx_opmode_oneshot);
}

bool QuadSoloSimClient::ReceiveDataFromRobot(DataFromQuadSim& rx_data)
{
	//std::cout << "fetching new data"<<std::endl;

	if(ReceiveGyroData(rx_data.imu_data.gyro) &&
			ReceiveAccData(rx_data.imu_data.acc) &&
			ReceiveQuadPosition(rx_data.pos_i) &&
			ReceiveQuadVelocity(rx_data.vel_i) &&
			ReceiveQuadOrientation(rx_data.rot_i) &&
			ReceiveQuadQuaternion(rx_data.quat_i))
		//&& Get3DScanPoints(rx_data->laser_points))
	{
		rx_data.rot_rate_b.x = rx_data.imu_data.gyro.x;
		rx_data.rot_rate_b.y = rx_data.imu_data.gyro.y;
		rx_data.rot_rate_b.z = rx_data.imu_data.gyro.z;

		//Get3DScanPoints(rx_data.laser_points);

		//		std::cout << "got it!"<<std::endl;
		return true;
	}
	else
		return false;
}

void QuadSoloSimClient::SendDataToRobot(const DataToQuadSim& rcmd)
{
	float front_left_prop, rear_right_prop, rear_left_prop, front_right_prop;

	front_left_prop = rcmd.ang_vel[0];
	front_right_prop = rcmd.ang_vel[1];
	rear_right_prop = rcmd.ang_vel[2];
	rear_left_prop = rcmd.ang_vel[3];

	// add limits
	if(front_left_prop > max_motor_speed_)
		front_left_prop = max_motor_speed_;
	if(rear_right_prop > max_motor_speed_)
		rear_right_prop = max_motor_speed_;
	if(rear_left_prop > max_motor_speed_)
		rear_left_prop = max_motor_speed_;
	if(front_right_prop > max_motor_speed_)
		front_right_prop = max_motor_speed_;

	if(front_left_prop < 0)
		front_left_prop = 0;
	if(rear_right_prop < 0)
		rear_right_prop = 0;
	if(rear_left_prop < 0)
		rear_left_prop = 0;
	if(front_right_prop < 0)
		front_right_prop = 0;

	//	std::cout << "prop cmd 0-4: "<<cmd.ang_vel[0] << " , "<< cmd.ang_vel[1] << " , " << cmd.ang_vel[2] << " , " << cmd.ang_vel[3] << std::endl;

	// send commands to simulator
	simxSetFloatSignal(client_id_, "propeller_cmd_front_left", front_left_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear_right", rear_right_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear_left", rear_left_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_front_right", front_right_prop, simx_opmode_oneshot);

	//	if(motor_update_result != 0)
	//		std::cout << "ERROR: Failed to update one or more motor commands." << std::endl;
}

bool QuadSoloSimClient::GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X])
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

bool QuadSoloSimClient::Get3DScanPoints(std::vector<Point3f>& points)
{
	bool result;

	points.clear();

	if (simxGetStringSignal(client_id_,"3d_scanner_points",&scannerptr_sig,&scannerptr_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		uint64_t cnt = scannerptr_sig_size/sizeof(float);

		for(uint64_t i = 0; i < cnt/3; i++)
		{
			Point3f pt;

			uint64_t pixel_idx = i*3;
			pt.x = ((float*)scannerptr_sig)[pixel_idx];
			pt.y = ((float*)scannerptr_sig)[pixel_idx + 1];
			pt.z = ((float*)scannerptr_sig)[pixel_idx + 2];

			points.push_back(pt);
		}

		//		std::cout << "3d scan ptr size: " << points.size() << std::endl;
		//
		//		for(uint64_t i = 0; i < 3; ++i)
		//		{
		//			std::cout << " ( " << points[i].x << "," << points[i].y << "," << points[i].z << " ) " << std::endl;
		//		}
		//		std::cout << " ------ " << std::endl;

		result = true;
	}
	else
		result = false;

	// TODO laser data is not mandatory now
	return true;
}

bool QuadSoloSimClient::ReceiveGyroData(IMU_DataType& data)
{
	if (simxGetStringSignal(client_id_,"solo_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=gyro_sig_size/4;

		if(cnt == 3)
		{
			data.x=((float*)gyro_sig)[0];
			data.y=((float*)gyro_sig)[1];
			data.z=((float*)gyro_sig)[2];
		}

		//		std::cout << "gyro data received" << std::endl;
		//		std::cout << "gyro (x, y, z) = " << "( " << data->x <<" , " << data->y << " , " << data->z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadSoloSimClient::ReceiveAccData(IMU_DataType& data)
{
	if (simxGetStringSignal(client_id_,"solo_acc",&acc_sig,&acc_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=acc_sig_size/4;

		if(cnt == 3)
		{
			data.x=((float*)acc_sig)[0];
			data.y=((float*)acc_sig)[1];
			data.z=((float*)acc_sig)[2];
		}

		//		std::cout << "acc data received" << std::endl;
		//		std::cout << "acc (x, y, z) = " << "( " << data->x <<" , " << data->y << " , " << data->z << " )" << std::endl;

		return true;
	}
	else
		return false;
}

bool QuadSoloSimClient::ReceiveQuadPosition(Point3f& data)
{
	if (simxGetObjectPosition(client_id_, ref_handle_, -1, quad_pos,simx_opmode_buffer) == simx_error_noerror)
	{
		data.x = quad_pos[0];
		data.y = quad_pos[1];
		data.z = quad_pos[2];

		//		std::cout << "pos data received" << std::endl;
		//		std::cout << "pos (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadSoloSimClient::ReceiveQuadVelocity(Point3f& data)
{
	if (simxGetObjectVelocity(client_id_, ref_handle_, quad_linear_vel, quad_angular_vel ,simx_opmode_buffer) == simx_error_noerror)
	{
		data.x = quad_linear_vel[0];
		data.y = quad_linear_vel[1];
		data.z = quad_linear_vel[2];

		//		std::cout << "vel data received" << std::endl;
		//		std::cout << "vel (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadSoloSimClient::ReceiveQuadOrientation(Point3f& data)
{if (simxGetObjectOrientation(client_id_, ref_handle_, -1, quad_ori, simx_opmode_buffer) == simx_error_noerror)
{
	data.x = quad_ori[0];
	data.y = quad_ori[1];
	data.z = quad_ori[2];

	//		std::cout << "ori data received" << std::endl;
	//		std::cout << "ori (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
	return true;
}
else
	return false;
}

bool QuadSoloSimClient::ReceiveQuadQuaternion(Quaternion& data)
{
	if (simxGetStringSignal(client_id_,"solo_quat",&quat_sig,&quat_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=acc_sig_size/4;

		if(cnt == 3)
		{
			data.x=((float*)quat_sig)[0];
			data.y=((float*)quat_sig)[1];
			data.z=((float*)quat_sig)[2];
			data.w=((float*)quat_sig)[3];

			//			std::cout << "quat data received" << std::endl;
			//			std::cout << "new quaternion: "<< data->w << " , "
			//					<< data->x << " , " << data->y << " , "
			//					<< data->z << std::endl;
		}
		return true;
	}
	else
		return false;
}
