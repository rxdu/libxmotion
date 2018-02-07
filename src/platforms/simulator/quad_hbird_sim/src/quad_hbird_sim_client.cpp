/* 
 * quad_hbird_sim_client.cpp
 * 
 * Created on: Sep 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <quad_hbird_sim/quad_hbird_sim_client.hpp>
#include <iostream>

using namespace librav;

QuadHbirdSimClient::QuadHbirdSimClient():
		VrepSimClient<DataFromQuadSim, DataToQuadSim>()
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

QuadHbirdSimClient::QuadHbirdSimClient(simxInt clientId):
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

QuadHbirdSimClient::~QuadHbirdSimClient()
{
	delete[] image_raw_;
	delete gyro_sig;
	delete acc_sig;
}

void QuadHbirdSimClient::ConfigDataStreaming(void)
{
	// get simulation object handles
	simxGetObjectHandle(client_id_, "asctec_hummingbird",&quad_handle_,simx_opmode_oneshot_wait);
	simxGetObjectHandle(client_id_, "ctrl_ref",&ref_handle_,simx_opmode_oneshot_wait);

	// initialize robot status data streaming
	simxGetObjectPosition(client_id_, ref_handle_, -1, quad_pos,simx_opmode_streaming);
	simxGetObjectVelocity(client_id_, ref_handle_, quad_linear_vel, quad_angular_vel,simx_opmode_streaming);
	simxGetObjectOrientation(client_id_, ref_handle_, -1, quad_ori, simx_opmode_streaming);

	simxGetStringSignal(client_id_,"hummingbird_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"hummingbird_quat",&quat_sig,&quat_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"hummingbird_acc",&acc_sig,&acc_sig_size,simx_opmode_streaming);
	simxGetStringSignal(client_id_,"3d_scanner_points",&scannerptr_sig,&scannerptr_sig_size,simx_opmode_streaming);

	// initialize motor values
	simxSetFloatSignal(client_id_, "propeller_cmd_front", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_right", 0.0, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_left", 0.0, simx_opmode_oneshot);
}

bool QuadHbirdSimClient::ReceiveDataFromSimRobot(DataFromQuadSim *rdata)
{
	//std::cout << "fetching new data"<<std::endl;

	if(ReceiveGyroData(rdata->imu_data.gyro) &&
			ReceiveAccData(rdata->imu_data.acc) &&
			ReceiveQuadPosition(rdata->pos_i) &&
			ReceiveQuadVelocity(rdata->vel_i) &&
			ReceiveQuadOrientation(rdata->rot_i) &&
			ReceiveQuadQuaternion(rdata->quat_i))
		//&& Get3DScanPoints(rx_data->laser_points))
	{
		rdata->rot_rate_b.x = rdata->imu_data.gyro.x;
		rdata->rot_rate_b.y = rdata->imu_data.gyro.y;
		rdata->rot_rate_b.z = rdata->imu_data.gyro.z;

		Get3DScanPoints(rdata->laser_points);

//		std::cout << "got it!"<<std::endl;
		return true;
	}
	else
		return false;
}

void QuadHbirdSimClient::UpdateCtrlLoop(const DataFromQuadSim &rdata, DataToQuadSimv *rcmd)
{
	quad_ctrl.UpdateRobotState(rdata);
	*rcmd = quad_ctrl.UpdateCtrlLoop();
}

void QuadHbirdSimClient::SendDataToSimRobot(const DataToQuadSim &rcmd)
{
	float front_prop, rear_prop, left_prop, right_prop;

	front_prop = rcmd.ang_vel[0];
	right_prop = rcmd.ang_vel[1];
	rear_prop = rcmd.ang_vel[2];
	left_prop = rcmd.ang_vel[3];

	// add limits
	if(front_prop > max_motor_speed_)
		front_prop = max_motor_speed_;
	if(rear_prop > max_motor_speed_)
		rear_prop = max_motor_speed_;
	if(left_prop > max_motor_speed_)
		left_prop = max_motor_speed_;
	if(right_prop > max_motor_speed_)
		right_prop = max_motor_speed_;

	if(front_prop < 0)
		front_prop = 0;
	if(rear_prop < 0)
		rear_prop = 0;
	if(left_prop < 0)
		left_prop = 0;
	if(right_prop < 0)
		right_prop = 0;

	//	std::cout << "prop cmd 0-4: "<<cmd.ang_vel[0] << " , "<< cmd.ang_vel[1] << " , " << cmd.ang_vel[2] << " , " << cmd.ang_vel[3] << std::endl;

	// send commands to simulator
	simxSetFloatSignal(client_id_, "propeller_cmd_front", front_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_rear", rear_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_left", left_prop, simx_opmode_oneshot);
	simxSetFloatSignal(client_id_, "propeller_cmd_right", right_prop, simx_opmode_oneshot);

	//	if(motor_update_result != 0)
	//		std::cout << "ERROR: Failed to update one or more motor commands." << std::endl;
}

bool QuadHbirdSimClient::GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X])
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

bool QuadHbirdSimClient::Get3DScanPoints(std::vector<Point3f>& points)
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

bool QuadHbirdSimClient::ReceiveGyroData(Point3f& data)
{
	if (simxGetStringSignal(client_id_,"hummingbird_gyro",&gyro_sig,&gyro_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=gyro_sig_size/4;

		if(cnt == 3)
		{
			data.x=((float*)gyro_sig)[0];
			data.y=((float*)gyro_sig)[1];
			data.z=((float*)gyro_sig)[2];
		}
//		std::cout << "gyro (x, y, z) = " << "( " << data->x <<" , " << data->y << " , " << data->z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadHbirdSimClient::ReceiveAccData(Point3f& data)
{
	if (simxGetStringSignal(client_id_,"hummingbird_acc",&acc_sig,&acc_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=acc_sig_size/4;

		if(cnt == 3)
		{
			data.x=((float*)acc_sig)[0];
			data.y=((float*)acc_sig)[1];
			data.z=((float*)acc_sig)[2];
		}
//		std::cout << "acc (x, y, z) = " << "( " << data->x <<" , " << data->y << " , " << data->z << " )" << std::endl;

		return true;
	}
	else
		return false;
}

bool QuadHbirdSimClient::ReceiveQuadPosition(Point3f& data)
{
	if (simxGetObjectPosition(client_id_, ref_handle_, -1, quad_pos,simx_opmode_buffer) == simx_error_noerror)
	{
		data.x = quad_pos[0];
		data.y = quad_pos[1];
		data.z = quad_pos[2];
//		std::cout << "pos (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadHbirdSimClient::ReceiveQuadVelocity(Point3f& data)
{
	if (simxGetObjectVelocity(client_id_, ref_handle_, quad_linear_vel, quad_angular_vel ,simx_opmode_buffer) == simx_error_noerror)
	{
		data.x = quad_linear_vel[0];
		data.y = quad_linear_vel[1];
		data.z = quad_linear_vel[2];
//		std::cout << "vel (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadHbirdSimClient::ReceiveQuadOrientation(Point3f& data)
{if (simxGetObjectOrientation(client_id_, ref_handle_, -1, quad_ori, simx_opmode_buffer) == simx_error_noerror)
	{
		data.x = quad_ori[0];
		data.y = quad_ori[1];
		data.z = quad_ori[2];
//		std::cout << "ori (x, y, z) = " << "( " << (*data).x <<" , " << (*data).y << " , " << (*data).z << " )" << std::endl;
		return true;
	}
	else
		return false;
}

bool QuadHbirdSimClient::ReceiveQuadQuaternion(Quaternion& data)
{
	if (simxGetStringSignal(client_id_,"hummingbird_quat",&quat_sig,&quat_sig_size,simx_opmode_buffer) == simx_error_noerror)
	{
		int cnt=quat_sig_size/4;

		if(cnt == 4)
		{
			data.x=((float*)quat_sig)[0];
			data.y=((float*)quat_sig)[1];
			data.z=((float*)quat_sig)[2];
			data.w=((float*)quat_sig)[3];

//			std::cout << "new quaternion: "<< data->w << " , "
//					<< data->x << " , " << data->y << " , "
//					<< data->z << std::endl;
		}
		return true;
	}
	else
		return false;
}
