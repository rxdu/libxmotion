/*
 * att_euler_con.h
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_ATT_EULER_CON_H_
#define CONTROL_ATT_EULER_CON_H_

#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl {

// input of attitude controller (using Euler)
typedef struct
{
	float euler_d[3];
	float rot_rate_d[3];
	float delta_w_F;
}AttEulerConInput;

// output of attitude controller
typedef struct{
	float motor_ang_vel_d[3];
}AttEulerConOutput;

class AttEulerCon
{
public:
	AttEulerCon();
	~AttEulerCon();

private:
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;

public:
	void Update(const QuadState& rs, const AttEulerConInput& input, AttEulerConOutput& output);
};

}

#endif /* CONTROL_ATT_EULER_CON_H_ */
