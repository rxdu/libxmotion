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
#include <map>

#include "graph/graph.hpp"
#include "road_map/road_map.hpp"
#include "traffic_map/traffic_elements.hpp"
#include "traffic_map/traffic_flow.hpp"

namespace librav
{
class TrafficMap
{
public:
  TrafficMap(std::shared_ptr<RoadMap> map);
  ~TrafficMap();

  void DiscretizeTrafficRegions(double resolution);

  std::vector<TrafficChannel> GetAllTrafficChannels();
  TrafficChannel GetTrafficChannel(std::string src, std::string dst);
  std::vector<TrafficChannel> FindConflictingChannels(std::string src, std::string dst);

  TrafficFlow *GetTrafficFlow(std::string source);
  std::vector<TrafficFlow *> GetAllTrafficFlows();

  std::vector<TrafficFlow *> FindConflictingFlows(std::string src, std::string dst);
  void CheckCollision(TrafficFlow *scflow, TrafficFlow *flow);

  std::vector<FlowTrackPoint> BackTrackCollision(TrafficFlow *scflow, TrafficFlow *flow, double v);
  std::vector<FlowTrackPoint> BackTrackCollision(TrafficFlow *scflow, std::vector<TrafficFlow *> flows, double v);

private:
  std::shared_ptr<RoadMap> road_map_;
  bool map_discretized_ = false;

  std::shared_ptr<Graph_t<TrafficRegion *>> graph_;
  std::unordered_map<int32_t, TrafficRegion *> traffic_regions_;
  std::map<std::pair<std::string, std::string>, TrafficChannel> traffic_channels_;
  std::map<std::string, TrafficFlow *> traffic_flows_;

  Polygon lane_block_footprint_;

  double DecomposeTrafficRegion(TrafficRegion *region, std::string last_region, double last_remainder, double resolution);
  VehiclePose InterpolatePose(SimplePoint pt0, SimplePoint pt1, double s);
  VehiclePose InterpolatePoseInversed(SimplePoint pt0, SimplePoint pt1, double s);
};
} // namespace librav

#endif /* TRAFFIC_FLOW_MAP_HPP */
