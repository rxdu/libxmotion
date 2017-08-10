/*
 * rc_car_sim_types.h
 *
 *  Created on: Aug 10, 2017
 *      Author: rdu
 */

#ifndef SIMULATOR_RC_CAR_SIM_TYPES_H_
#define SIMULATOR_RC_CAR_SIM_TYPES_H_

#include "common/control_types.h"
#include "control/rc_car_ctrl/estimator/rc_car_state.h"
#include "control/rc_car_ctrl/rc_car_config.h"

namespace librav
{

typedef struct
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	IMUData imu_data;

	float body_vel;
	float driving_vel_right;
	float driving_vel_left;
	float steering_ang;
} DataFromRCCarSim;

typedef struct
{
	RCCarCmd cmd;
} DataToRCCarSim;
}

#endif /* SIMULATOR_RC_CAR_SIM_TYPES_H_ */
