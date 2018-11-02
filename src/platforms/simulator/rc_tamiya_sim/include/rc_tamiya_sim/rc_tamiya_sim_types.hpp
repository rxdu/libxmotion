/* 
 * rc_tamiya_sim_types.hpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RC_TAMIYA_SIM_TYPES_HPP
#define RC_TAMIYA_SIM_TYPES_HPP

#include "common/librav_types.hpp"
#include "rc_tamiya_sim/rc_tamiya_sim_config.hpp"

namespace librav
{
struct DataFromRCTamiyaSim
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	IMU6DOFData imu_data;

	float body_vel;
	float driving_vel_right;
	float driving_vel_left;
	float steering_ang;
};

struct DataToRCTamiyaSim
{
	float driving_vel_cmd;
	float steering_ang_cmd;
};
}

#endif /* RC_TAMIYA_SIM_TYPES_HPP */
