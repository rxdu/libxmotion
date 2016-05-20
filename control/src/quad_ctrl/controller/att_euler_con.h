/*
 * att_euler_con.h
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_ATT_EULER_CON_H_
#define CONTROL_ATT_EULER_CON_H_

#include "quad_ctrl/controller/controller_base.h"

namespace srcl_ctrl {

class AttEulerCon: public Controller
{
public:
	AttEulerCon(RobotState *_rs);
	~AttEulerCon();

private:
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;

public:
	void Update(ControlInput *input, ControlOutput *cmd);
};

}

#endif /* CONTROL_ATT_EULER_CON_H_ */
