/* 
 * motion_model.hpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <memory>
#include <string>

#include "graph/graph.hpp"
#include "road_network/road_map.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_ranking/lane_block.hpp"
#include "threat_field/collision_field.hpp"

namespace librav
{
struct MMStateEst
{
  MMStateEst() : position_x(0), position_y(0), velocity_x(0), velocity_y(0),
                 sigma_px(0), sigma_py(0), sigma_vx(0), sigma_vy(0) {}

  MMStateEst(double px, double py,
             double vx, double vy,
             double sig_px, double sig_py,
             double sig_vx, double sig_vy) : position_x(px), position_y(py), velocity_x(vx), velocity_y(vy),
                                             sigma_px(sig_px), sigma_py(sig_py), sigma_vx(sig_vx), sigma_vy(sig_vy) {}
  MMStateEst(double px, double py,
             double vx, double vy,
             double sig_p, double sig_v) : position_x(px), position_y(py), velocity_x(vx), velocity_y(vy),
                                           sigma_px(sig_p), sigma_py(sig_p), sigma_vx(sig_v), sigma_vy(sig_v) {}

  double position_x;
  double position_y;
  double velocity_x;
  double velocity_y;

  double sigma_px;
  double sigma_py;
  double sigma_vx;
  double sigma_vy;
};

class MotionPoint
{
public:
  MotionPoint(MMStateEst est) : estimate_(est) {}

  MMStateEst estimate_;
  DenseGridPixel grid_position_;

  void SetGridCoordinate(DenseGridPixel pos)
  {
    grid_position_ = pos;
  }
};

/// Motion model manages the evolution of detected traffic participants
class MotionModel
{
public:
  MotionModel(std::shared_ptr<RoadMap> map);
  ~MotionModel() = default;

  void AddVehicleStateEstimate(MMStateEst pt) { ests_.push_back(pt); }
  void SetVehicleStateEstimates(std::vector<MMStateEst> pts) { ests_ = pts; }

  void MergePointsToNetwork();
  void GenerateCollisionField();
  Eigen::MatrixXd GetThreatFieldVisMatrix();

private:
  std::shared_ptr<RoadMap> road_map_;
  std::shared_ptr<Graph_t<LaneBlock>> line_network_;

  std::vector<MMStateEst> ests_;
  std::shared_ptr<CollisionField> cfield_;

  std::vector<MotionPoint> points_;

  void ConstructLineNetwork();
  void FindMotionChains(std::string start_lane);
};
} // namespace librav

#endif /* MOTION_MODEL_HPP */
