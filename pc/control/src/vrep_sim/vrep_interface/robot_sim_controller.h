/*
 * robot_sim_controller.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_
#define CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_

//extern "C" {
//    #include "extApi.h"
///*	#include "extApiCustom.h" // custom remote API functions */
//}
//
//#include "common/robot_state_base.h"

#include <cstdint>

namespace srcl_ctrl
{

template<typename DataFromSimType, typename DataToSimType, typename RobotStateType, typename RobotCmdType>
class RobotSimController
{
protected:
	RobotSimController():ctrl_loop_count_(0){};
	virtual ~RobotSimController(){};

protected:
	RobotStateType rs_;
	uint64_t ctrl_loop_count_;

public:
	virtual const RobotStateType& GetRobotState() { return rs_;};
	virtual const DataToSimType ConvertRobotCmdToSimCmd(const RobotCmdType& cmd) = 0;

	virtual void UpdateRobotState(DataFromSimType* data) = 0;
	virtual RobotCmdType UpdateCtrlLoop() = 0;
	virtual RobotCmdType UpdateCtrlLoop(const RobotStateType& desired){};
};

}

#endif /* CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROLLER_H_ */
