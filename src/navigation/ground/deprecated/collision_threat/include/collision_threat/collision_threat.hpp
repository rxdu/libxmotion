/* 
 * collision_threat.hpp
 * 
 * Created on: Aug 29, 2018 00:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef COLLISION_THREAT_HPP
#define COLLISION_THREAT_HPP

#include <cmath>
#include <memory>

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

#include "threat_field/collision_field.hpp"

namespace librav
{
struct CollisionRegion
{
  CollisionRegion(VehiclePose p, double spd, double time) : pose(p), speed(spd), time_stamp(time)
  {
    velocity_x = speed * std::cos(p.theta);
    velocity_y = speed * std::sin(p.theta);
  }

  VehiclePose pose;
  double speed;
  double time_stamp;
  
  double velocity_x;
  double velocity_y;
  double sigma_x = 3.0;
  double sigma_y = 0.5;
};

class DynamicThreatModel
{
public:
  DynamicThreatModel(std::shared_ptr<RoadMap> map, std::shared_ptr<TrafficMap> tmap);

  void SetEgoTrafficFlow(TrafficFlow *flow) { ego_flow_ = flow; }

  std::shared_ptr<CollisionField> EvaluateThreat(double ave_spd);

  std::vector<TrafficFlow *> GetConflictingFlows() { return conflicting_flows_; }
  double CalculateThreatLevel(std::shared_ptr<CollisionField> cfield, double posx, double posy, double velx, double vely);

private:
  std::shared_ptr<RoadMap> road_map_;
  std::shared_ptr<TrafficMap> traffic_map_;
  std::vector<CollisionRegion> regions_;
  std::vector<FlowTrackPoint> track_points_;

  TrafficFlow *ego_flow_;
  std::vector<TrafficFlow *> conflicting_flows_;

  void GenerateCollisionRegions(const std::vector<FlowTrackPoint> &centers, double spd);
  std::shared_ptr<CollisionField> GenerateCollisionField();
};
} // namespace librav

#endif /* COLLISION_THREAT_HPP */
