/* 
 * vehicle_motion.hpp
 * 
 * Created on: Nov 06, 2018 07:31
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_MOTION_HPP
#define VEHICLE_MOTION_HPP

#include "traffic_sim/vehicle_info.hpp"
#include "cav_common/vehicle_state.hpp"

namespace librav
{
class VehicleMotion
{
  public:
    VehicleMotion(VehicleInfo info);

    bool IsOutOfScope() const { return out_of_scope_; }
    VehicleState GetStateAt(double t);

  private:
    VehicleInfo vehicle_info_;
    bool out_of_scope_ = false;

    VehicleState PropagateConstSpeedModel(double t);
};
} // namespace librav

#endif /* VEHICLE_MOTION_HPP */
