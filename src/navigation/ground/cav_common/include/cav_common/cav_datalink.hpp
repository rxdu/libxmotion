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

namespace librav
{
struct CAV_COMMON_CHANNELS
{
    static constexpr auto VEHICLE_ESTIMATIONS_CHANNEL = "vehicle_estimations";
};

struct CAV_SIM_CHANNELS
{
    static constexpr auto SIM_SYNC_TRIGGER_CHANNEL = "sim_sync_trigger";
};
} // namespace librav

#endif /* CAV_DATALINK_HPP */
