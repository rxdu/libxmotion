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
#include "threat_ranking/motion_state.hpp"

namespace librav
{
class MotionModel;

class MotionPoint
{
public:
  MotionPoint() = default;
  MotionPoint(MMStateEst est) : estimate_(est) {}

  MMStateEst estimate_;
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

  std::shared_ptr<CollisionField> GenerateCollisionField();
  std::shared_ptr<CollisionField> GeneratePredictedCollisionField(double t);

  std::vector<MMStatePrediction> PropagateMotionChains(double t);

private:
  std::vector<MMStateEst> ests_;
  // std::shared_ptr<CollisionField> cfield_;

  // one state estimation maps to one motion point in the model
  std::vector<MotionPoint> points_;
  // the motion of a point is tracked by using a motion chain
  std::vector<std::shared_ptr<MotionChain>> chains_;

  void ConstructLineNetwork();
};
} // namespace librav

#endif /* MOTION_MODEL_HPP */
