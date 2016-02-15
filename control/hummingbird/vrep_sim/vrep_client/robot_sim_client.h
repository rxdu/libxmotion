/*
 * robot_sim_client.h
 *
 *  Created on: Aug 4, 2015
 *      Author: rdu
 */

#ifndef VREP_CLIENT_ROBOT_SIM_CLIENT_H_
#define VREP_CLIENT_ROBOT_SIM_CLIENT_H_

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include <vrep_client/robot_datatypes.h>

namespace srcl_ctrl
{
class RobotSimClient
{
public:
	RobotSimClient(simxInt clientId):client_id_(clientId){};
	virtual ~RobotSimClient(){};

public:
	virtual bool ReceiveDataFromRobot(DataFromRobot *rstate) = 0;
	virtual void SendDataToRobot(const DataToRobot &rcmd) = 0;

protected:
	virtual void ConfigDataStreaming(void) = 0;

protected:
	simxInt client_id_;
};
}

#endif /* VREP_CLIENT_ROBOT_SIM_CLIENT_H_ */
