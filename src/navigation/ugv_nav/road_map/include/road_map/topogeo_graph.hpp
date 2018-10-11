/* 
 * topogeo_graph.hpp
 * 
 * Created on: Jul 25, 2018 06:53
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TOPOGEO_GRAPH_HPP
#define TOPOGEO_GRAPH_HPP

#include <memory>
#include <string>
#include <cstdint>
#include <vector>
#include <unordered_map>

#include "graph/graph.hpp"
#include "road_map/lane_block.hpp"

namespace librav
{
class RoadMap;

class TopoGeoGraph
{
public:
  TopoGeoGraph() = delete;
  TopoGeoGraph(RoadMap *map);
  ~TopoGeoGraph();

  std::vector<std::string> sinks_;
  std::vector<std::string> sources_;

  std::vector<std::string> FindInteractingLanes(std::vector<std::string> names);
  std::vector<std::string> FindInteractingLanes(std::vector<int32_t> ids);

private:
  RoadMap *road_map_;
  std::shared_ptr<Graph_t<LaneBlock *>> graph_;
  std::unordered_map<int32_t, LaneBlock *> lane_blocks_;

  void ConstructGraph();
  std::vector<std::string> BacktrackVertices(int32_t id);
};
} // namespace librav

#endif /* TOPOGEO_GRAPH_HPP */
