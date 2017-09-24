#ifndef PATH_REPAIR_SIM_VIRTUAL_QUAD_H_
#define PATH_REPAIR_SIM_VIRTUAL_QUAD_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "quadrotor/path_repair/sim/sim_path_repair.h"
#include "quadrotor/path_repair/sim/sim_depth_sensor.h"

namespace librav
{

class VirtualQuadrotor
{
public:
  VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm);

  void Load_5by5_Config();
  void Load_30by50_Config();
  void Load_10by10_Config();

public:
  bool IsReady();
  void Step();

private:
  std::shared_ptr<lcm::LCM> lcm_;
  std::shared_ptr<SimDepthSensor> dsensor_;
  std::shared_ptr<SimPathRepair> qplanner_;

  // pose info for sim reset
  Position2D init_pos_;
  int32_t init_height_;

  // pose info for flight sim
  Position2D current_pos_;
  int32_t current_height_;
  double current_heading_;

  // path from planner
  SimPath active_path_;
  double traveled_distance_;

  void MoveForward();
  void PublishState();
};
}

#endif /* PATH_REPAIR_SIM_VIRTUAL_QUAD_H_ */
