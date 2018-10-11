/* 
 * vrep_sim_client.hpp
 * 
 * Created on: Aug 4, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 * 
 */

#ifndef VREP_SIM_CLIENT_HPP
#define VREP_SIM_CLIENT_HPP

extern "C" {
#include "vrep_api/remoteApi/extApi.h"
}

namespace librav
{

template <typename DataFromSimType, typename DataToSimType>
class VrepSimClient
{
  public:
	simxInt client_id_;

	/* 
	 * The behavior of the following functions need to be defined for a specific robot.
	 * 
	 * User does NOT need to call these functions explicitly. They will be called by RobotSimProcess class
	 * 	in the following order inside StartSimLoop_Synchronous() or StartSimLoop_ASynchronous() function:
	 * 	(Pseudo code)
	 *
	 * 	DataFromSimType sensor_data;
	 * 	DataToSimType robot_cmd;
	 * 	ConfigDataStreaming();
	 *  Other_config();
	 * 	while(connected)
	 * 	{
	 * 		ReceiveDataFromSimRobot(&sensor_data);
	 * 		UpdateCtrlLoop(sensor_data, &robot_cmd);
	 * 		SendDataToSimRobot(robot_cmd);
	 * 		Other_house_keeping();
	 * 	}
	 * 
	 */
	virtual void ConfigDataStreaming(void) = 0;
	virtual bool ReceiveDataFromSimRobot(DataFromSimType *rdata) = 0;
	virtual void UpdateCtrlLoop(const DataFromSimType &rdata, DataToSimType *rcmd) = 0;
	virtual void SendDataToSimRobot(const DataToSimType &rcmd) = 0;

  protected:
	VrepSimClient(simxInt id = -1) : client_id_(id){};
	virtual ~VrepSimClient() = default;
};
}

#endif /* VREP_SIM_CLIENT_HPP */
