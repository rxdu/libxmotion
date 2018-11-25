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
extern "C"
{
#include "vrep_api/remoteApi/extApi.h"
}

#include "vrep_interface/vrep_sim_client.hpp"
#include "vrep_interface/stopwatch/stopwatch.h"

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

    // by default run as fast as possible
    void StartSimLoop_Synchronous(int32_t loop_period_ms = 0)
    {
        sync_mode_ = true;
        StartSimLoop(loop_period_ms);
    }

    void StartSimLoop_ASynchronous(int32_t loop_period_ms = 20)
    {
        sync_mode_ = false;
        StartSimLoop(loop_period_ms);
    }

  private:
    const uint64_t server_port_;
    bool server_connected_;
    bool sync_mode_;
    uint64_t loop_count_;

    DataFromSimType data_from_sim_;
    DataToSimType data_to_sim_;

    stopwatch::StopWatch stopwatch_;

    std::shared_ptr<VrepSimClient<DataFromSimType, DataToSimType>> sim_client_;

    void StartSimLoop(int32_t loop_period_ms)
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

            // send trigger to simulator
            if (sync_mode_)
            {
                simxSynchronousTrigger(sim_client_->client_id_);
                // after this call, the first simulation step is finished (Blocking function call)
                simxGetPingTime(sim_client_->client_id_, &ping_time);
            }

            if (loop_period_ms > 0)
                stopwatch_.sleep_until_ms(loop_period_ms);

            // every time a trigger is sent, simulation time forwards loop_period_ms ms
            loop_count_++;
        }

        std::cout << "INFO: Exited control loop." << std::endl;

        simxStopSimulation(sim_client_->client_id_, simx_opmode_oneshot_wait);

        simxFinish(sim_client_->client_id_);
    }
};
} // namespace librav

#endif /* VREP_SIM_PROCESS_HPP */
