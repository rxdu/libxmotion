/*
 * quad_types.h
 *
 *  Created on: Aug 9, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_TYPES_H_
#define CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_TYPES_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include <common/control_types.h>
//#include "quad_ctrl/estimator/quad_state.h"

namespace srcl_ctrl
{

typedef struct
{
	// input of position controller
	float pos_d[3];
	float vel_d[3];
	float acc_d[3];
	float yaw_d;
	float yaw_rate_d;

	// input of attitude controller (using Euler)
	float euler_d[3];
	float rot_rate_d[3];
	float delta_w_F;

	// input of attitude controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float ftotal_d;

}ControlInput;

typedef struct{
	// output of position controller (using Euler)
	float euler_d[3];
	float delta_w_F;

	// output of position controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float ftotal_d;

	// output of attitude controller
	float motor_ang_vel_d[3];
}ControlOutput;

typedef struct
{
	float ang_vel[4];
}QuadCmd;

enum class QuadFlightType {
	X_TYPE,
	PLUS_TYPE,
};

#define IMG_RES_X 160
#define IMG_RES_Y 90

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

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
} DataFromQuad;

typedef struct
{
	QuadCmd motor_cmd;
} DataToQuad;

}

#endif /* CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_TYPES_H_ */
