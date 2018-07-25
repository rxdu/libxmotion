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

namespace librav
{
enum class LaneBockType
{
    LaneSegment = 0,
    LaneConnector
};

struct LaneBlock
{
    LaneBlock() : id(-1), name("null"){};
    LaneBlock(int32_t _id, std::string _name = "default") : id(_id), name(_name){};
    LaneBlock(int32_t _id, LaneBockType _type, std::string _name = "default") : id(_id), name(_name), type(_type){};

    int32_t id;
    std::string name;
    LaneBockType type;

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
