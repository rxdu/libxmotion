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

#include <iostream>

#include "geometry/polygon.hpp"
#include "traffic_map/traffic_elements.hpp"
#include "tree/unique_tree.hpp"

namespace librav
{

///////////////////////////////////////////////////////////////////////////
struct FlowUnit
{
  FlowUnit(Polygon py) : footprint(py) {}

  Polygon footprint;
  double time_label = 0.0;
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

/// Traffic flow model: one source to multiple sinks
class TrafficFlow
{
public:
  TrafficFlow() = delete;
  TrafficFlow(TrafficChannel channel);
  TrafficFlow(std::vector<TrafficChannel> channels);
  ~TrafficFlow() = default;

  UniqueTree<FlowUnit> flow_tree_;

  void AssignSpeedProfile();

  std::vector<TrafficFlow> CheckConflicts(const std::vector<TrafficFlow> &flows);
  void LabelConflictBlocks(TrafficFlow *other);

  std::vector<Polygon> GetAllLaneBlocks();
  std::vector<Polygon> GetConflictingLaneBlocks();

private:
  std::string source_;
  std::vector<TrafficChannel> channels_;

  UniqueTree<FlowUnit> BuildTree(const std::vector<TrafficChannel> &chns);

  void CheckCollision(UniqueTree<FlowUnit> *tree, const FlowUnit &other);
};
} // namespace librav

#endif /* TRAFFIC_FLOW_HPP */
