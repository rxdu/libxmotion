/*
 * robot_sim_controller.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_
#define CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include "common/robot_state_base.h"

namespace srcl_ctrl
{

template<typename DataFromSimType, typename DataToSimType, typename RobotStateType>
class RobotSimController
{
protected:
	RobotSimController();
	~RobotSimController();

private:
	RobotStateType rs_;

public:
	void UpdateRobotState(DataFromSimType* data) = 0;
	void UpdateCtrlLoop(const RobotStateType& desired) = 0;
};

}

#endif /* CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_ */
