/*
 * rc_car_client.h
 *
 *  Created on: Aug 4, 2015
 *      Author: rdu
 */

#ifndef RCCAR_CLIENT_H_
#define RCCAR_CLIENT_H_

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include <vrep_client/robot_datatypes.h>
#include <vrep_client/robot_sim_client.h>

#define MAX_JOINT1_TORQUE 100
#define MAX_JOINT2_TORQUE 100

namespace RobotToolkitRIVeR{

class RCCarClient: public RobotSimClient
{
public:
	RCCarClient(simxInt clientId);
	~RCCarClient();

public:
	bool ReceiveDataFromRobot(DataFromRobot *rstate);
	void SendDataToRobot(const DataToRobot &rcmd);

private:
	void ConfigDataStreaming(void);

	bool GetCarDrivingVel(float *rvel, float *lvel, float *body_vel);
	bool GetCarSteeringAngle(float *data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);

	void SetCarSteeringAngle(const float &cmd);
	void SetCarDrivingVel(const float &rcmd, const float &lcmd);

private:
	simxInt car_handle_;
	simxInt camera_handle_;
	simxInt steering_right_;
	simxInt steering_left_;
	simxInt driving_front_right_;
	simxInt	driving_front_left_;
	simxInt driving_rear_right_;
	simxInt driving_rear_left_;

	float body_lin_vel_[3];
	float body_ang_vel_[3];
	float body_vel_;

	float driving_right_vel_;
	float driving_left_vel_;
	float steering_angle_;
	float steering_vel_;

	simxUChar *image_raw_;
	int img_res[2];
};

}

#endif /* QUADSIM_CLIENT_H_ */
