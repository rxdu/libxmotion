/*
 * rc_car_sim_client.h
 *
 *  Created on: Aug 10, 2017
 *      Author: rdu
 */

#ifndef SIMULATOR_RC_CAR_SIM_CLIENT_H_
#define SIMULATOR_RC_CAR_SIM_CLIENT_H_

#include <cstdint>

#include "common/librav_types.hpp"

#include "rc_car_sim/rc_car_sim_types.h"
#include "vrep_sim/vrep_interface/robot_sim_client.h"

namespace librav {

class RCCarSimClient : public RobotSimClient<DataFromRCCarSim, DataToRCCarSim>
{
public:
	RCCarSimClient();
	~RCCarSimClient();

private:
	virtual void ConfigDataStreaming(void);

public:
	virtual bool ReceiveDataFromRobot(DataFromRCCarSim& rstate);
	virtual void SendDataToRobot(const DataToRCCarSim& rcmd);

	bool GetCarDrivingVel(float& rvel, float& lvel, float& body_vel);
	bool GetCarSteeringAngle(float& data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);

	void SetCarSteeringAngle(float cmd);
	void SetCarSteeringVelocity(float cmd);
	void SetCarDrivingVel(float rcmd, float lcmd);

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

#endif /* SIMULATOR_RC_CAR_SIM_CLIENT_H_ */
