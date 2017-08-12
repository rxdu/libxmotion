/*
 * quad_hbird_sim_types.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef SIMULATOR_QUAD_HBIRD_SIM_TYPES_H_
#define SIMULATOR_QUAD_HBIRD_SIM_TYPES_H_

#include "common/control_types.h"
#include "quad_ctrl/state/quad_config.h"

namespace librav {

struct DataFromQuadSim
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
};

struct DataToQuadSim
{
	float ang_vel[4];
};

}

#endif /* SIMULATOR_QUAD_HBIRD_SIM_TYPES_H_ */
