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
struct LatticeGraph
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
    
    static std::shared_ptr<Graph<LatticeNode, StateLattice>> Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start, int32_t expansion_iter = 2);
};
} // namespace librav

#endif /* LATTICE_GRAPH_HPP */
