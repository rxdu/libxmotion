/*
 * sim_process.h
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#ifndef SIM_PROCESS_SIM_PROCESS_H_
#define SIM_PROCESS_SIM_PROCESS_H_

#include <vrep_client/robot_sim_client.h>
#include <vrep_client/robot_datatypes.h>

namespace RobotToolkitRIVeR
{

class SimProcess
{
public:
	SimProcess(RobotSimClient *client): sim_client_(client){};
	virtual ~SimProcess(){};

public:
	virtual void SimLoopUpdate() = 0;
	bool ReceiveDataFromSimulator(){return sim_client_->ReceiveDataFromRobot(&rs_m);};
	void SendDataToSimulator(){sim_client_->SendDataToRobot(cmd_m);};

protected:
	RobotSimClient *sim_client_;
	DataFromRobot rs_m;
	DataToRobot cmd_m;
};

}

#endif /* SIM_PROCESS_SIM_PROCESS_H_ */
