/*
 * controller.h
 *
 *  Created on: Jul 28, 2015
 *      Author: rdu
 */

#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include <vrep_client/robot_datatypes.h>
#include "navigation/robot_state.h"

namespace srcl_ctrl
{

typedef struct
{
	float pos_d[3];
	float vel_d[3];
	float euler_d[3];
	float rot_rate_d[3];
	float delta_w_F;
}ControlInput;

typedef struct{
	// output of position controller
	float euler_d[3];
	float delta_w_F;

	// output of attitude controller
	float ang_vel_d[3];
}ControlOutput;

class Controller
{
public:
	Controller(RobotState* rs_ref):rs_(rs_ref){};
	virtual ~Controller(){};

protected:
	RobotState *rs_;

public:
	virtual void Update(ControlInput *input, ControlOutput *cmd) = 0;
};

}

#endif /* CONTROL_CONTROLLER_H_ */
