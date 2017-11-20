/* 
 * robot_sim_client.h
 * 
 * Created on: Aug 4, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef ROBOT_SIM_CLIENT_H
#define ROBOT_SIM_CLIENT_H

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

namespace librav
{

template<typename DataFromSimType, typename DataToSimType>
class RobotSimClient
{
public:
	simxInt client_id_;

	// the following functions need to be defined for a specific robot
	virtual void ConfigDataStreaming(void) = 0;
	virtual bool ReceiveDataFromRobot(DataFromSimType& state) = 0;
	virtual void SendDataToRobot(const DataToSimType& rcmd) = 0;

protected:
	RobotSimClient(simxInt id):client_id_(id){};
	RobotSimClient():client_id_(-1){};
	virtual ~RobotSimClient(){};
};

}

#endif /* ROBOT_SIM_CLIENT_H */
