/* 
 * vrep_sim_process.hpp
 * 
 * Created on: Sep 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VREP_SIM_PROCESS_HPP
#define VREP_SIM_PROCESS_HPP

#include <cstdint>
#include <memory>
#include <vector>

// headers for vrep remote api
extern "C" {
#include "vrep_api/remoteApi/extApi.h"
}

#include "vrep_interface/vrep_sim_client.hpp"

namespace librav
{

template <typename DataFromSimType, typename DataToSimType>
class VrepSimProcess
{
  public:
	VrepSimProcess(std::shared_ptr<VrepSimClient<DataFromSimType, DataToSimType>> client) : server_port_(29999),
																							  server_connected_(false),
																							  sync_mode_(true),
																							  loop_count_(0),
																							  sim_client_(client){};
	virtual ~VrepSimProcess() = default;

	bool ConnectToServer(int32_t comm_thread_cycle_ms = 5, uint64_t port = -1)
	{
		if (port == -1)
			port = server_port_;

		sim_client_->client_id_ = simxStart((simxChar *)"127.0.0.1", port, true, true, 2000, comm_thread_cycle_ms);

		if (sim_client_->client_id_ != -1)
		{
			server_connected_ = true;
		}
		else
			server_connected_ = false;

		return server_connected_;
	}

	void StartSimLoop_Synchronous()
	{
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

	std::shared_ptr<VrepSimClient<DataFromSimType, DataToSimType>> sim_client_;

	void StartSimLoop()
	{
		sim_client_->ConfigDataStreaming();

		if (sync_mode_)
		{
			simxSynchronous(sim_client_->client_id_, true);
			std::cout << "INFO: Enabled synchronous mode." << std::endl;
		}
		simxStartSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

		simxInt ping_time = 0;

		while (simxGetConnectionId(sim_client_->client_id_) != -1)
		{
			if (loop_count_ == 0)
				std::cout << "INFO: Entered control loop, client id: " << sim_client_->client_id_ << std::endl;

			// update simulated control loop
			// if (!sim_client_->ReceiveDataFromSimRobot(&data_from_sim_) && sync_mode_)
			// 	std::cout << "failed to fetch new data" << std::endl;
			sim_client_->ReceiveDataFromSimRobot(&data_from_sim_);

			// call user-defined update function
			sim_client_->UpdateCtrlLoop(data_from_sim_, &data_to_sim_);

			// send command to robot
			sim_client_->SendDataToSimRobot(data_to_sim_);

			// delay to slow down the simulation
			// extApi_sleepMs(1);
			// usleep(50);

			// send trigger to simulator
			if (sync_mode_)
			{
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

#endif /* VREP_SIM_PROCESS_HPP */
