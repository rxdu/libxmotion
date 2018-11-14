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
#include <vector>
#include <iostream>

#include "graph/graph.hpp"
#include "decomp/curvilinear_grid.hpp"

#include "traffic_map/traffic_map.hpp"
#include "state_lattice/state_lattice.hpp"

namespace librav
{
class LatticeGraph
{
  public:
    struct LatticeNode : CurvilinearCell
    {
        LatticeNode(int64_t _id) : CurvilinearCell(-1, -1, id){};
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

    static std::shared_ptr<Graph<LatticeNode, StateLattice>> Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter = 2);
    static std::vector<StateLattice> Search(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter = 2);
    static std::shared_ptr<Graph<LatticeNode, StateLattice>> Search(std::vector<StateLattice> &path, std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter = 2);

  private:
    static std::shared_ptr<Graph<LatticeNode, StateLattice>> Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter, std::vector<int32_t> &final_nodes);

    // lattice expansion horizon
    static constexpr int32_t min_h = 6;
    static constexpr int32_t max_h = min_h;
};
} // namespace librav

#endif /* LATTICE_GRAPH_HPP */
