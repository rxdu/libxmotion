/* 
 * traffic_sim_manager.hpp
 * 
 * Created on: Dec 05, 2018 21:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_SIM_MANAGER_HPP
#define TRAFFIC_SIM_MANAGER_HPP

#include <memory>

#include "datalink/lcm_link.hpp"
#include "stopwatch/stopwatch.h"

#include "traffic_map/map_loader.hpp"

#include "traffic_sim/traffic_sim_config.hpp"
#include "traffic_sim/vehicle_manager.hpp"

namespace librav
{
class TrafficSimManager
{
  public:
    TrafficSimManager(TrafficSimConfig config);

    bool ValidateSimConfig();
    void RunSim(bool sync_mode = false);

  private:
    TrafficSimConfig config_;

    std::shared_ptr<LCMLink> data_link_;
    stopwatch::StopWatch sim_stopwatch_;
    bool data_link_ready_ = false;
    bool sync_trigger_ready_ = false;

    MapLoader map_loader_;
    VehicleManager vehicle_manager_;

    void UpdateSimState(double t);

    void HandleLCMMessage_SyncTrigger(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::SimSyncTrigger *msg);
};
} // namespace librav

#endif /* TRAFFIC_SIM_MANAGER_HPP */
