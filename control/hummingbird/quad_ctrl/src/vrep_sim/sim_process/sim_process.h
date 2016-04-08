/*
 * sim_process.h
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#ifndef SIM_PROCESS_SIM_PROCESS_H_
#define SIM_PROCESS_SIM_PROCESS_H_

#include "vrep_client/robot_sim_client.h"
#include "common/robot_datatypes.h"

namespace srcl_ctrl
{

class SimProcess
{
public:
	SimProcess(RobotSimClient *client): sim_client_(client){};
	virtual ~SimProcess(){};

public:
	virtual void SimLoopUpdate() = 0;
	bool ReceiveDataFromSimulator(){return sim_client_->ReceiveDataFromRobot(&rs_m_);};
	void SendDataToSimulator(){sim_client_->SendDataToRobot(cmd_m_);};

protected:
	RobotSimClient *sim_client_;
	DataFromRobot rs_m_;
	DataToRobot cmd_m_;
};

}

#endif /* SIM_PROCESS_SIM_PROCESS_H_ */
