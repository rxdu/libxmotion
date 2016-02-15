/*
 * rc_car_client.h
 *
 *  Created on: Aug 4, 2015
 *      Author: rdu
 */

#ifndef QUADSIM_CLIENT_H_
#define QUADSIM_CLIENT_H_

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include <vrep_client/robot_datatypes.h>
#include <vrep_client/robot_sim_client.h>

#define MAX_JOINT1_TORQUE 100
#define MAX_JOINT2_TORQUE 100

namespace srcl_ctrl{

class QuadSimClient: public RobotSimClient
{
public:
	QuadSimClient(simxInt clientId);
	~QuadSimClient();

public:
	bool ReceiveDataFromRobot(DataFromRobot *rstate);
	void SendDataToRobot(const DataToRobot &rcmd);

private:
	void ConfigDataStreaming(void);

	bool ReceiveQuadPosition(void);
	bool ReceiveQuadPosition(Position *data);
	bool ReceiveGyroData(IMU_DataType *data);
	bool ReceiveAccData(IMU_DataType *data);
	void SendPropellerCmd(QuadCmd cmd);

	bool GetCarDrivingVel(float *rvel, float *lvel, float *body_vel);
	bool GetCarSteeringAngle(float *data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);

	void SetCarSteeringAngle(const float &cmd);
	void SetCarDrivingVel(const float &rcmd, const float &lcmd);

private:
	simxInt quad_handle_;

private:
	IMUData imu_data;
	simxFloat quad_pos[3];
	simxUChar* gyro_sig;
	simxInt gyro_sig_size;
	simxUChar* acc_sig;
	simxInt acc_sig_size;
	simxInt camera_handle_;

	simxUChar *image_raw_;
	int img_res[2];
};

}

#endif /* QUADSIM_CLIENT_H_ */
