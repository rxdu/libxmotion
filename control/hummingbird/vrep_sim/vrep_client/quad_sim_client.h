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
#define MAX_MOTOR_SPEED	10000

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

	bool ReceiveGyroData(IMU_DataType *data);
	bool ReceiveAccData(IMU_DataType *data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);

	bool ReceiveQuadPosition(Point3 *data);
	bool ReceiveQuadVelocity(Point3 *data);
	bool ReceiveQuadOrientation(Point3 *data);

	void SendPropellerCmd(QuadCmd cmd);

private:
	simxInt quad_handle_;
	simxInt ref_handle_;

private:
	// quadrotor kinematics/dynamics
	IMUData imu_data;
	simxFloat quad_pos[3];
	simxFloat quad_linear_vel[3];
	simxFloat quad_angular_vel[3];
	simxFloat quad_ori[3];
	simxUChar* gyro_sig;
	simxInt gyro_sig_size;
	simxUChar* acc_sig;
	simxInt acc_sig_size;

	// vision
	simxInt camera_handle_;
	simxUChar *image_raw_;
	int img_res[2];
};

}

#endif /* QUADSIM_CLIENT_H_ */
