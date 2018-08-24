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

#include "geometry/polygon.hpp"
#include "traffic_map/traffic_elements.hpp"
#include "tree/unique_tree.hpp"

namespace librav
{

///////////////////////////////////////////////////////////////////////////

struct FlowUnit
{
  Polygon footprint;

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
};

/// Traffic flow model: one source to multiple sinks
class TrafficFlow
{
public:
  TrafficFlow() = default;
  TrafficFlow(std::string src, std::vector<TrafficChannel> channels);
  ~TrafficFlow() = default;

  std::vector<Polygon> GetAllLaneBlocks() const;
  void CheckConflicts(const TrafficChannel &other);

  UniqueTree<FlowUnit> BuildTree();

private:
  std::string source_;
  std::vector<TrafficChannel> channels_;
};
} // namespace librav

#endif /* TRAFFIC_FLOW_HPP */
