/*
 * pos_quat_con.h
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#ifndef CONTROL_POS_QUAT_CON_H_
#define CONTROL_POS_QUAT_CON_H_

#include "control/controller.h"

namespace srcl_ctrl {

class PosQuatCon: public Controller
{
public:
	PosQuatCon(RobotState *_rs);
	~PosQuatCon();

private:
	float kp_0;
	float ki_0;
	float kd_0;
	float kp_1;
	float ki_1;
	float kd_1;
	float kp_2;
	float ki_2;
	float kd_2;

	double pos_e_integral[3];
	double zint_uppper_limit;
	double zint_lower_limit;
	double xyint_uppper_limit;
	double xyint_lower_limit;

public:
	void Update(ControlInput *input, ControlOutput *cmd);
};

}



#endif /* CONTROL_POS_QUAT_CON_H_ */
