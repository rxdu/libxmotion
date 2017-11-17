/* 
 * robot_sim_control.h
 * 
 * Created on: Sep 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef ROBOT_SIM_CONTROL_H
#define ROBOT_SIM_CONTROL_H

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

#endif /* ROBOT_SIM_CONTROL_H */
