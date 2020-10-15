/* 
 * cav_datalink.hpp
 * 
 * Created on: Dec 05, 2018 23:01
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAV_DATALINK_HPP
#define CAV_DATALINK_HPP

#include <string>

#include "datalink/lcm_link.hpp"

namespace ivnav
{
struct CAV_COMMON_CHANNELS
{
    static constexpr auto EGO_VEHICLE_STATE_CHANNEL = "ego_vehicle_state";
    static constexpr auto VEHICLE_ESTIMATIONS_CHANNEL = "vehicle_estimations";

    static constexpr auto CAV_MISSION_INFO_CHANNEL = "cav_mission_info";

    static constexpr auto DESIRED_TRAJECTORY_CHANNEL = "desired_trajectory";
};

struct CAV_SIM_CHANNELS
{
    static constexpr auto SIM_SYNC_TRIGGER_CHANNEL = "sim_sync_trigger";
};
} // namespace ivnav

#endif /* CAV_DATALINK_HPP */
