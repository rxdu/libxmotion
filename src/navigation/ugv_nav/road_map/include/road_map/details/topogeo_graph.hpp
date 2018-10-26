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
#include "geometry/polyline.hpp"

namespace librav
{
class RoadMap;

enum class LaneBlockType
{
    TopoConnected = 0,
    GeoConnected
};

struct LaneBlock
{
    LaneBlock() : id(-1), name("null"){}
    LaneBlock(int32_t _id, std::string _name = "default") : id(_id), name(_name) {}
    LaneBlock(int32_t _id, LaneBlockType _type, std::string _name = "default") : id(_id), name(_name), type(_type) {}

    int32_t id;
    std::string name;
    LaneBlockType type = LaneBlockType::TopoConnected;
    Polyline center_line;

    int64_t GetUniqueID() const
    {
        return id;
    }

    bool operator==(const LaneBlock &other)
    {
        if (other.id == this->id)
            return true;
        else
            return false;
    }
};

class TopoGeoGraph
{
public:
  TopoGeoGraph() = delete;
  TopoGeoGraph(RoadMap *map);
  ~TopoGeoGraph();

  std::vector<std::string> sinks_;
  std::vector<std::string> sources_;
  std::vector<std::string> isolated_lanes_;

  std::vector<std::string> FindConflictingLanes(std::vector<std::string> names);
  std::vector<std::string> FindConflictingLanes(std::vector<int32_t> ids);

  bool HasOnlyOneSubsequentLane(std::string name);

private:
  RoadMap *road_map_;
  std::shared_ptr<Graph<LaneBlock *>> graph_;
  std::unordered_map<int32_t, LaneBlock *> lane_blocks_;

  void ConstructGraph();
  std::vector<std::string> BacktrackVertices(int32_t id);
};
} // namespace librav

#endif /* TOPOGEO_GRAPH_HPP */
