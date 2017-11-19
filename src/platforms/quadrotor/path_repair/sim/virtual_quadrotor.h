#ifndef PATH_REPAIR_SIM_VIRTUAL_QUAD_H_
#define PATH_REPAIR_SIM_VIRTUAL_QUAD_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "path_repair/sim/sim_path_repair.h"
#include "path_repair/sim/sim_depth_sensor.h"
#include "utility/logging/logger.h"

namespace librav
{

class VirtualQuadrotor
{
public:
  VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm);

  void SetMapSize(int32_t map_x, int32_t map_y, int32_t map_z);
  void SetConfig(int32_t map_x, int32_t map_y, int32_t map_z, int32_t height, int32_t sensor_rng);
  void Load_5by5_Config();
  void Load_10by10_Config();
  void Load_15by20_Config();
  void Load_20by25_Config();
  void Load_20by30_Config();  
  void Load_30by50_Config();  
  void Load_50by50_Config();
  void Load_45by60_Config();

public:
  bool IsReady();
  void Step();
  void CmpStep();
  void SetSensorRange(int32_t rng);
  void SetSensorFOV(double fov);
  void SetInitPosition(Position2Di pos, int32_t hei);
  void SetGoalPosition(Position2Di pos, int32_t hei);

private:
  std::shared_ptr<lcm::LCM> lcm_;
  std::shared_ptr<SimDepthSensor> dsensor_;
  std::shared_ptr<SimPathRepair> qplanner_;

  // pose info for sim reset
  Position2Di init_pos_;
  int32_t init_height_;

  // pose info for flight sim
  Position2Di current_pos_;
  int32_t current_height_;
  double current_heading_;

  // path from planner
  SimPath active_path_;  

  // data recording
  double traveled_distance_;  
  bool init_path_found_;
  double init_repair_path_cost_;
  int64_t run_flag_;
  int64_t sim_index_;
  int64_t sim_steps_;
  std::unique_ptr<CsvLogger> logger_;

  bool EvaluationPath(const SimPath& old_path, const SimPath& new_path);
  void MoveForward(bool enable_path_repair = true);
  void PublishState();
  double CalcWaypointDistance(Position2Di pos1, Position2Di pos2);
};
}

#endif /* PATH_REPAIR_SIM_VIRTUAL_QUAD_H_ */
