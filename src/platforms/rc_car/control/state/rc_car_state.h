/*
 * car_state.h
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#ifndef ROBOT_STATE_CAR_STATE_H_
#define ROBOT_STATE_CAR_STATE_H_

#include <cstdint>
#include "control/car_ctrl/state/rc_car_config.h"

namespace librav
{

struct RCCarSensorData
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	IMUData imu_data;

	float body_vel;
	float driving_vel_right;
	float driving_vel_left;
	float steering_ang;
};

struct RCCarCmd
{
	float driving_vel_rcmd;
	float driving_vel_lcmd;
	float steering_ang_cmd;
};

//template<typename T>
class RCCarState
{
public:
	RCCarState();
	~RCCarState();

private:
	float driving_velocity_;
	//uint8_t mono_image_[IMG_RES_Y][IMG_RES_X];

public:
	void UpdateRobotState(const RCCarSensorData &new_data);
};

}

#endif /* ROBOT_STATE_CAR_STATE_H_ */
