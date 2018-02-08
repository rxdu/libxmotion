/* 
 * vehicle_sim.hpp
 * 
 * Created on: Nov 20, 2017 10:14
 * Description: a simple continuous-time car model
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_SIM_HPP
#define VEHICLE_SIM_HPP

#include "common/librav_types.hpp"
#include "traffic/traffic_participant.hpp"

namespace librav
{

class VehicleSim : public TrafficParticipant
{
public:
  VehicleSim();
  VehicleSim(Position2Dd init_pos);
  ~VehicleSim() = default;

public:
  void Update(double t, double dt) override;
  
private:
  Position2Dd init_pos_;
};

}

#endif /* VEHICLE_SIM_HPP */
