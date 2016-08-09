/*
 * quad_ctrl_io.h
 *
 *  Created on: Aug 9, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUADCON_IO_H_
#define CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUADCON_IO_H_

#include <common/control_types.h>
#include "quad_ctrl/estimator/robot_state.h"

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
}

#endif /* CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUADCON_IO_H_ */
