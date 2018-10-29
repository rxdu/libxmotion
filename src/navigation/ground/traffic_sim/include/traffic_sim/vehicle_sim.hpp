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

namespace librav
{

class VehicleSim
{
public:
  VehicleSim() = default;
  VehicleSim(double x, double y, double spd);
  ~VehicleSim() = default;

  void Update(double t);
  
private:
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double speed_ = 0.0;
};

}

#endif /* VEHICLE_SIM_HPP */
