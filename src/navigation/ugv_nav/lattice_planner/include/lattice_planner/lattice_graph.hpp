/* 
 * lattice_graph.hpp
 * 
 * Created on: Aug 08, 2018 23:43
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_GRAPH_HPP
#define LATTICE_GRAPH_HPP

#include <memory>
#include <cstdint>
#include <ostream>

#include "graph/graph.hpp"
#include "decomp/curvilinear_grid.hpp"

#include "road_map/road_map.hpp"
#include "state_lattice/state_lattice.hpp"

namespace librav
{
struct LatticeNode : CurvilinearCell
{
    LatticeNode(CurvilinearCell *cell, std::shared_ptr<TrafficChannel> chn) : CurvilinearCell(*cell), channel(chn)
    {
        auto pt = channel->grid_->ConvertToCurvePoint(center);
        state = MotionState(pt.x, pt.y, pt.theta, pt.kappa);
    }

    MotionState state;
    std::shared_ptr<TrafficChannel> channel;

    friend std::ostream &operator<<(std::ostream &os, const LatticeNode &node)
    {
        os << "Lattice node: " << node.state.x << " , " << node.state.y << " , " << node.state.theta << " , " << node.state.kappa;
        return os;
    }
};

class LatticeGraph : public Graph<LatticeNode, StateLattice>
{
  public:
    LatticeGraph();

    void Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start, int32_t expansion_iter = 2);

  private:
    // std::shared_ptr<RoadMap> road_map_;
    // std::shared_ptr<GraphType> graph_;
};
} // namespace librav

#endif /* LATTICE_GRAPH_HPP */
