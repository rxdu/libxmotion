/* 
 * traffic_flow_map.hpp
 * 
 * Created on: Aug 20, 2018 22:25
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_MAP_HPP
#define TRAFFIC_FLOW_MAP_HPP

#include <memory>

#include "graph/graph.hpp"
#include "road_map/road_map.hpp"

namespace librav
{
struct FlowRegion
{
  FlowRegion() : id(-1), name("null") {}
  explicit FlowRegion(int32_t _id, std::string _name = "default") : id(_id), name(_name) {}

  int32_t id;
  std::string name;
  Polyline center_line;

  int64_t GetUniqueID() const
  {
    return id;
  }

  bool operator==(const FlowRegion &other)
  {
    if (other.id == this->id)
      return true;
    else
      return false;
  }
};

class TrafficFlowMap
{
public:
  TrafficFlowMap(std::shared_ptr<RoadMap> map);
  ~TrafficFlowMap();

private:
  std::shared_ptr<RoadMap> road_map_;

  std::shared_ptr<Graph_t<FlowRegion *>> graph_;
  std::unordered_map<int32_t, FlowRegion *> flow_regions_;

  void ConstructLaneGraph();
  void DecomposeCenterlines();
};
} // namespace librav

#endif /* TRAFFIC_FLOW_MAP_HPP */
