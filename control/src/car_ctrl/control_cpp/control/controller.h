/*
 * controller.h
 *
 *  Created on: Jul 28, 2015
 *      Author: rdu
 */

#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include <vrep_client/robot_datatypes.h>
#include "robot_state/robot_state.h"

namespace RobotToolkitRIVeR
{

class Controller
{
public:
	Controller(RobotState* rs_ref):rs_(rs_ref){};
	virtual ~Controller(){};

protected:
	RobotState *rs_;

public:
	virtual void Update(const VehicleState &rs_d, VehicleCmd *cmd) = 0;
};

}

#endif /* CONTROL_CONTROLLER_H_ */
