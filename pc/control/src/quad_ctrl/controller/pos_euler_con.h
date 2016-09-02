/*
 * pos_euler_con.h
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_POS_EULER_CON_H_
#define CONTROL_POS_EULER_CON_H_

#include <controller/quad_types.h>
#include "quad_ctrl/controller/controller_base.h"

namespace srcl_ctrl{

class PosEulerCon: public Controller {
public:
	PosEulerCon(QuadState *_rs);
	~PosEulerCon();

private:
	float kp_0;
	float kd_0;
	float kp_1;
	float kd_1;
	float kp_2;
	float kd_2;

public:
	void Update(ControlInput *input, ControlOutput *cmd);
};

}


#endif /* CONTROL_POS_EULER_CON_H_ */
