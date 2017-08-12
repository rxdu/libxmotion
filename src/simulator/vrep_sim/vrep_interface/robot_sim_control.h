/*
 * robot_sim_control.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROL_H_
#define CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROL_H_

//extern "C" {
//    #include "extApi.h"
///*	#include "extApiCustom.h" // custom remote API functions */
//}
//
//#include "common/robot_state_base.h"

#include <cstdint>

namespace librav
{

template<typename DataFromSimType, typename DataToSimType, typename RobotStateType, typename RobotCmdType>
class RobotSimControl
{
public:
	virtual const RobotStateType& GetRobotState() { return rs_;};
	virtual DataToSimType ConvertRobotCmdToSimCmd(const RobotCmdType& cmd) = 0;

	virtual void UpdateRobotState(const DataFromSimType& data) = 0;
	virtual RobotCmdType UpdateCtrlLoop() = 0;

protected:
	RobotSimControl():ctrl_loop_count_(0){};
	virtual ~RobotSimControl() = default;	

	RobotStateType rs_;
	RobotStateType est_rs_;
	uint64_t ctrl_loop_count_;	
};

}

#endif /* CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_CONTROL_H_ */
