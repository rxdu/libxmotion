/*
 * controller.h
 *
 *  Created on: Jul 28, 2015
 *      Author: rdu
 */

#ifndef CONTROL_CONTROLLER_BASE_H_
#define CONTROL_CONTROLLER_BASE_H_

#include "common/control_types.h"
#include "quad_ctrl/controller/quadcon_io.h"
#include "quad_ctrl/estimator/robot_state.h"

namespace srcl_ctrl
{
class Controller
{
public:
	Controller(RobotState* rs_ref):rs_(rs_ref){};
	virtual ~Controller(){};

protected:
	const RobotState *rs_;

public:
	virtual void Update(ControlInput *input, ControlOutput *cmd) = 0;
};

}

#endif /* CONTROL_CONTROLLER_BASE_H_ */
