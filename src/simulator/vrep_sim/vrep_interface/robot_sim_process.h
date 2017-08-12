/*
 * robot_sim_process.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_
#define CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_

#include <cstdint>
#include <memory>
#include <vector>

// headers for vrep remote api
extern "C" {
    #include "extApi.h"
//	#include "extApiCustom.h" // custom remote API functions
}

#include "vrep_sim/vrep_interface/robot_sim_client.h"
#include "vrep_sim/vrep_interface/robot_sim_control.h"

namespace librav {

template<typename DataFromSimType, typename DataToSimType, typename RobotStateType, typename RobotCmdType>
class RobotSimProcess
{
public:
	RobotSimProcess(std::shared_ptr<RobotSimClient<DataFromSimType, DataToSimType>> client,
			std::shared_ptr<RobotSimControl<DataFromSimType, DataToSimType,RobotStateType, RobotCmdType>> controller):
		server_port_(29999),
		server_connected_(false),
		sync_mode_(true),
		loop_count_(0),
		sim_client_(client),
		robot_controller_(controller){};
	virtual ~RobotSimProcess() = default;

	bool ConnectToServer(uint64_t port = -1)
	{
		if(port == -1)
			port = server_port_;

		sim_client_->client_id_ = simxStart((simxChar*)"127.0.0.1",port,true,true,2000,5);

		if (sim_client_->client_id_ != -1) {
			server_connected_ = true;
		}
		else
			server_connected_ = false;

		return server_connected_;
	}

	void StartSimLoop_Synchronous(){
		sync_mode_ = true;
		StartSimLoop();
	}

	void StartSimLoop_ASynchronous()
	{
		sync_mode_ = false;
		StartSimLoop();
	}

private:
	const uint64_t server_port_;
	bool server_connected_;
	bool sync_mode_;
	uint64_t loop_count_;

	DataFromSimType data_from_sim_;
	DataToSimType data_to_sim_;

	std::shared_ptr<RobotSimClient<DataFromSimType, DataToSimType>> sim_client_;
	std::shared_ptr<RobotSimControl<DataFromSimType, DataToSimType,RobotStateType, RobotCmdType>> robot_controller_;

	void StartSimLoop()
	{
		sim_client_->ConfigDataStreaming();

		if(sync_mode_)
		{
			simxSynchronous(sim_client_->client_id_,true);
			std::cout << "INFO: Enabled synchronous mode." << std::endl;
		}
		simxStartSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

		simxInt ping_time = 0;

		while (simxGetConnectionId(sim_client_->client_id_) != -1)
		{
			if(loop_count_ == 0)
				std::cout << "INFO: Entered control loop, client id: " << sim_client_->client_id_ << std::endl;

			// update simulated control loop
			if(sim_client_->ReceiveDataFromRobot(data_from_sim_))
			{
				RobotCmdType rcmd;
				robot_controller_->UpdateRobotState(data_from_sim_);

				rcmd = robot_controller_->UpdateCtrlLoop();

				data_to_sim_ = robot_controller_->ConvertRobotCmdToSimCmd(rcmd);
			}
			else
				std::cout << "failed to fetch new data" << std::endl;

			// send command to robot
			sim_client_->SendDataToRobot(data_to_sim_);

			// delay to slow down the simulation
			// extApi_sleepMs(1);
			// usleep(50);

			// send trigger to simulator
			if(sync_mode_) {
				simxSynchronousTrigger(sim_client_->client_id_);
				// after this call, the first simulation step is finished (Blocking function call)
				simxGetPingTime(sim_client_->client_id_, &ping_time);
			}

			// every time a trigger is sent, simulation time forwards 10ms
			loop_count_++;
		}

		std::cout << "INFO: Exited control loop." << std::endl;

		simxStopSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

		simxFinish(sim_client_->client_id_);
	}
};

}

#endif /* CONTROL_SRC_VREP_SIM_VREP_INTERFACE_ROBOT_SIM_PROCESS_H_ */
