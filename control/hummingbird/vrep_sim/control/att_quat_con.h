/*
 * att_quat_con.h
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#ifndef CONTROL_ATT_QUAT_CON_H_
#define CONTROL_ATT_QUAT_CON_H_

#include "control/controller.h"

namespace srcl_ctrl {

class AttQuatCon: public Controller
{
public:
	AttQuatCon(RobotState *_rs);
	~AttQuatCon();

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

#endif /* CONTROL_ATT_QUAT_CON_H_ */
