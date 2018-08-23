/* 
 * traffic_map.hpp
 * 
 * Created on: Aug 20, 2018 22:25
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_MAP_HPP
#define TRAFFIC_FLOW_MAP_HPP

#include <memory>
#include <vector>

#include "graph/graph.hpp"
#include "road_map/road_map.hpp"
#include "traffic_flow.hpp"

namespace librav
{
struct TrafficRegion
{
  TrafficRegion() : id(-1), name("null") {}
  explicit TrafficRegion(int32_t _id, std::string _name = "default") : id(_id), name(_name) {}

  int32_t id;
  std::string name;
  Polyline center_line;

  bool discretized = false;
  double remainder = 0.0;
  std::vector<VehiclePose> anchor_points;

  int64_t GetUniqueID() const
  {
    return id;
  }

  bool operator==(const TrafficRegion &other)
  {
    if (other.id == this->id)
      return true;
    else
      return false;
  }
};

class TrafficMap
{
public:
  TrafficMap(std::shared_ptr<RoadMap> map);
  ~TrafficMap();

  // std::vector<Polygon> DecomposeCenterlines(std::vector<std::string> lanelets, double step = 1.0);

// private:
public:
  std::shared_ptr<RoadMap> road_map_;

  std::shared_ptr<Graph_t<TrafficRegion *>> graph_;
  std::unordered_map<int32_t, TrafficRegion *> flow_regions_;

  Polygon lane_block_footprint_;

  void ConstructLaneGraph();
  std::vector<Polygon> DiscretizeRoadNetwork(double resolution);

  std::vector<Polygon> DecomposeTrafficRegion(TrafficRegion *region, double last_remainder, double resolution);
  VehiclePose InterpolatePose(SimplePoint pt0, SimplePoint pt1, double s);
  VehiclePose InterpolatePoseInversed(SimplePoint pt0, SimplePoint pt1, double s);
};
} // namespace librav

#endif /* TRAFFIC_FLOW_MAP_HPP */
