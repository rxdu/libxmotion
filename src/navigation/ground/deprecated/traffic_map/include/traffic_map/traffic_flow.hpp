/* 
 * traffic_flow.hpp
 * 
 * Created on: Aug 22, 2018 09:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_HPP
#define TRAFFIC_FLOW_HPP

#include <cmath>
#include <iostream>

#include "geometry/polygon.hpp"
#include "tree/unique_tree.hpp"

#include "traffic_map/traffic_elements.hpp"

namespace librav
{

struct FlowUnit
{
  FlowUnit(Polygon py) : footprint(py) {}
  FlowUnit(Polygon py, VehiclePose ps) : footprint(py), pose(ps) {}

  Polygon footprint;
  VehiclePose pose;
  double time_stamp = 0.0;
  bool in_collision = false;

  bool operator==(const FlowUnit &other)
  {
    if (footprint.GetPointNumer() != other.footprint.GetPointNumer())
      return false;
    for (int32_t i = 0; i < footprint.GetPointNumer(); ++i)
    {
      auto pt1 = footprint.GetPoint(i);
      auto pt2 = other.footprint.GetPoint(i);

      if ((pt1.x != pt2.x) || (pt1.y != pt2.y))
        return false;
    }
    return true;
  }

  double CalculateDistance(const FlowUnit &other)
  {
    double err_x = pose.x - other.pose.x;
    double err_y = pose.y - other.pose.y;

    return std::hypot(err_x, err_y);
  }

  friend std::ostream &operator<<(std::ostream &os, const FlowUnit &unit)
  {
    double x = 0;
    double y = 0;
    for (int32_t i = 0; i < unit.footprint.GetPointNumer(); ++i)
    {
      x += unit.footprint.GetPoint(i).x;
      y += unit.footprint.GetPoint(i).y;
    }
    os << "(x,y): " << x / unit.footprint.GetPointNumer() << " , " << y / unit.footprint.GetPointNumer();
    return os;
  }
};

struct FlowTrackPoint
{
  FlowTrackPoint() : time_stamp(0) {}
  FlowTrackPoint(VehiclePose ps, double time) : pose(ps), time_stamp(time) {}
  FlowTrackPoint(VehiclePose ops, VehiclePose ps, double time) : origin(ops), pose(ps), time_stamp(time) {}

  VehiclePose origin;
  VehiclePose pose;
  double time_stamp;
};

/// Traffic flow model: one source to multiple sinks
class TrafficFlow
{
  using FlowNode = UniqueTree<FlowUnit>::NodeType;

public:
  TrafficFlow() = delete;
  TrafficFlow(TrafficChannel channel);
  TrafficFlow(std::vector<TrafficChannel> channels);
  ~TrafficFlow() = default;

  void AssignConstantSpeedProfile(int32_t start_id, double dt);

  std::string GetFlowSource() { return source_; }
  std::string GetSingleSourceFlowSink() { return channels_.front().sink; }

  std::vector<Polygon> GetAllLaneBlocks();
  std::vector<Polygon> GetConflictingLaneBlocks();
  std::vector<FlowUnit> GetConflictingFlowUnits() { return collision_units_; }

  std::vector<FlowTrackPoint> BackTrackSingleChannelCollision(TrafficFlow *flow, double v);

private:
  std::string source_;
  std::vector<TrafficChannel> channels_;
  UniqueTree<FlowUnit> flow_tree_;
  std::vector<FlowUnit> collision_units_;

  UniqueTree<FlowUnit> BuildTree(const std::vector<TrafficChannel> &chns);
  void LabelUnitCollision(FlowUnit *unit);

  FlowTrackPoint BackTrackFlowUnit(const FlowUnit &start, double v, double t);
  VehiclePose InterpolatePose(VehiclePose pt0, VehiclePose pt1, double s);
  VehiclePose InterpolatePoseInversed(VehiclePose pt0, VehiclePose pt1, double s);
};
} // namespace librav

#endif /* TRAFFIC_FLOW_HPP */
