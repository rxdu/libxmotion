/*
 * robot_sim_process.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_
#define CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_

#include <cstdint>

// headers for vrep remote api
extern "C" {
    #include "extApi.h"
//	#include "extApiCustom.h" // custom remote API functions
}

#include "vrep_sim/vrep_interface/robot_sim_client.h"

namespace srcl_ctrl {

template<typename DataFromSimType, typename DataToSimType>
class RobotSimProcess
{
public:
	RobotSimProcess(RobotSimClient<DataFromSimType, DataToSimType>* client):
		server_port_(29999),
		sim_client_(client),
		server_connected_(false),
		loop_count_(0){};
	virtual ~RobotSimProcess(){};

private:
	const uint64_t server_port_;
	RobotSimClient<DataFromSimType, DataToSimType>* sim_client_;
	bool server_connected_;

	uint64_t loop_count_;

//public:
//	virtual void SimLoopUpdate() = 0;
//	bool ReceiveDataFromSimulator(){return sim_client_->ReceiveDataFromRobot(&rs_m_);};
//	void SendDataToSimulator(){sim_client_->SendDataToRobot(cmd_m_);};
//
//protected:
//	RobotSimClient *sim_client_;
//	DataFromQuad rs_m_;
//	DataToQuad cmd_m_;

public:
	bool ConnectToServer(uint64_t port = -1)
	{
		if(port == -1)
			port = server_port_;

		sim_client_->client_id_ = simxStart((simxChar*)"127.0.0.1",port,true,true,2000,5);

		if (sim_client_->client_id_!=-1)
			server_connected_ = true;
		else
			server_connected_ = false;

		return server_connected_;
	}

	void StartSimLoop_Synchronous(){
		simxSynchronous(sim_client_->client_id_,true);
		simxStartSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

		std::cout << "INFO: Enabled synchronous mode." << std::endl;

		simxInt ping_time = 0;

		while (simxGetConnectionId(sim_client_->client_id_)!=-1)
		{
			if(loop_count_ == 0)
				std::cout << "INFO: Entered control loop." << std::endl;

//			// update simulated control loop
//			if(sim_process.ReceiveDataFromSimulator())
//			{
//				// fetch the latest trajectory waypoint
//				UAVTrajectoryPoint pt;
//				pt = motion_server.GetCurrentDesiredPose();
//
//				// if no new point, stay where it was
//				if(!pt.point_empty)
//				{
//					sim_process.SimLoopUpdate(pt);
//					last_state = pt;
//				}
//				else
//					sim_process.SimLoopUpdate(last_state);
//			}
//			//			else
//			//				std::cout<<"failed to fetch data from simulator"<<std::endl;
//
//			// send command to robot
//			sim_process.SendDataToSimulator();

			// extApi_sleepMs(1);
			// usleep(50);

			// send trigger to simulator
			simxSynchronousTrigger(sim_client_->client_id_);

			// every time a trigger is sent, simulation time forwards 10ms
			loop_count_++;

			// after this call, the first simulation step is finished (Blocking function call)
			simxGetPingTime(sim_client_->client_id_, &ping_time);
		}

		std::cout << "INFO: Exited control loop." << std::endl;

		simxStopSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

		simxFinish(sim_client_->client_id_);
	}
};

}

#endif /* CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_ */
