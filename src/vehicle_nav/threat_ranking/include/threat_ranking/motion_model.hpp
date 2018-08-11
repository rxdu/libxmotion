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
#include <iostream>

#include "graph/graph.hpp"
#include "decomp/dense_grid.hpp"
#include "road_map/road_map.hpp"
#include "geometry/polyline.hpp"

#include "threat_field/traffic_participant.hpp"
#include "threat_field/collision_field.hpp"

#include "threat_ranking/lane_block.hpp"

namespace librav
{
class MotionModel;
class MotionChain;

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

//-----------------------------------------------------------------------------------//

struct MMStatePrediction
{
  MMStatePrediction(MMStateEst s, double px, double py, double vx, double vy) : base_state(s), position_x(px), position_y(py), velocity_x(vx), velocity_y(vy) {}

  MMStateEst base_state;

  double position_x;
  double position_y;
  double velocity_x;
  double velocity_y;

  bool operator==(const MMStatePrediction &other)
  {
    if (this->position_x == other.position_x &&
        this->position_y == other.position_y &&
        this->velocity_x == other.velocity_x &&
        this->velocity_y == other.velocity_y)
      return true;
    else
      return false;
  }

  bool operator!=(const MMStatePrediction &other)
  {
    if (*this == other)
      return false;
    else
      return true;
  }

  friend std::ostream &operator<<(std::ostream &os, const MMStatePrediction &data)
  {
    os << data.position_x << " , " << data.position_y << " , " << data.velocity_x << " , " << data.velocity_y;
    return os;
  }
};

//-----------------------------------------------------------------------------------//

class MotionPoint
{
public:
  MotionPoint() = default;
  MotionPoint(MMStateEst est) : estimate_(est) {}

  MMStateEst estimate_;
  DenseGridPixel grid_position_;

  void SetGridCoordinate(DenseGridPixel pos)
  {
    grid_position_ = pos;
  }
};

//-----------------------------------------------------------------------------------//

class MChainLink
{
public:
  MChainLink(int32_t id, Polyline ln);
  ~MChainLink();

  int32_t lanelet_id_;

  MotionPoint point_;
  Polyline polyline_;
  double length_;

  MChainLink *parent_link = nullptr;
  std::vector<MChainLink *> child_links;

private:
  double GetLength();
};

class MotionChain
{
public:
  MotionChain(MotionModel *model, MotionPoint pt, int32_t start);
  ~MotionChain();

  void FindStartingPoint();
  std::vector<MMStatePrediction> Propagate(double t);

private:
  int32_t start_id_;
  int32_t chain_depth_;
  int32_t chain_width_;

  int32_t start_segment_idx_ = 0;
  double dist_before_start_ = 0;

  MotionModel *model_;
  MotionPoint mp_pt_;
  MChainLink *base_link_;
  std::vector<MChainLink *> leaf_links_;
  std::vector<Polyline> chain_polylines_;

  void TraverseChain();
  void CreateChainPolylines();
  double GetPointLineDistance(SimplePoint ln_pt0, SimplePoint ln_pt1, SimplePoint pt);
  MMStatePrediction CalculatePrediction(SimplePoint pt0, SimplePoint pt1, double dist);
};

//-----------------------------------------------------------------------------------//

/// Motion model manages the evolution of detected traffic participants
class MotionModel
{
public:
  MotionModel(std::shared_ptr<RoadMap> map);
  ~MotionModel() = default;

  std::shared_ptr<RoadMap> road_map_;
  std::shared_ptr<Graph_t<LaneBlock>> cline_graph_;

  void AddVehicleStateEstimate(MMStateEst pt) { ests_.push_back(pt); }
  void SetVehicleStateEstimates(std::vector<MMStateEst> pts) { ests_ = pts; }

  void MergePointsToNetwork();
  void GenerateCollisionField();
  void GeneratePredictedCollisionField(double t);

  std::vector<MMStatePrediction> PropagateMotionChains(double t);

  Eigen::MatrixXd GetThreatFieldVisMatrix();

private:
  std::vector<MMStateEst> ests_;
  std::shared_ptr<CollisionField> cfield_;

  std::vector<MotionPoint> points_;
  std::vector<std::shared_ptr<MotionChain>> chains_;

  void ConstructLineNetwork();
};
} // namespace librav

#endif /* MOTION_MODEL_HPP */
