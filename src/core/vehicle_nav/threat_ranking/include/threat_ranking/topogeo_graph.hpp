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

#include "graph/graph.hpp"
#include "threat_ranking/lane_block.hpp"

namespace librav
{
class TopoGeoGraph
{
  public:
    TopoGeoGraph() = default;
    ~TopoGeoGraph() = default;

    void GenerateGraph();
    std::vector<std::string> BacktrackVertices(int32_t id);
    std::vector<std::string> FindInteractingLanes(std::vector<int32_t> ids);

  private:
    std::shared_ptr<Graph_t<LaneBlock>> graph_;
};
} // namespace librav

#endif /* TOPOGEO_GRAPH_HPP */
