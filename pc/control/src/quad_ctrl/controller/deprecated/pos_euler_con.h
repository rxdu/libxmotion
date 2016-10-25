/*
 * pos_euler_con.h
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_POS_EULER_CON_H_
#define CONTROL_POS_EULER_CON_H_

#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl{

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
}PosEulerConInput;

typedef struct{
	// output of position controller (using Euler)
	float euler_d[3];
	float delta_w_F;
}PosEulerConOutput;

class PosEulerCon{
public:
	PosEulerCon();
	~PosEulerCon();

private:
	float kp_0;
	float kd_0;
	float kp_1;
	float kd_1;
	float kp_2;
	float kd_2;

public:
	void Update(const QuadState& rs, const PosEulerConInput& input, PosEulerConOutput& output);
};

}


#endif /* CONTROL_POS_EULER_CON_H_ */
