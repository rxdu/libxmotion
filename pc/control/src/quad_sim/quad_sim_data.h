/*
 * quad_sim_data.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_SIM_QUAD_SIM_DATA_H_
#define CONTROL_SRC_QUAD_SIM_QUAD_SIM_DATA_H_

#include "common/control_types.h"

namespace srcl_ctrl {

typedef struct
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	std::vector<Point3f> laser_points;
	IMUData imu_data;

	// data only available in simulator
	Point3f pos_i;
	Point3f vel_i;
	Point3f rot_i;
	Quaternion quat_i;
	Point3f rot_rate_b;
} QuadDataFromSim;

typedef struct
{
	QuadCmd motor_cmd;
} QuadDataToSim;


}

#endif /* CONTROL_SRC_QUAD_SIM_QUAD_SIM_DATA_H_ */
