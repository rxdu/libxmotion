/* 
 * lattice_graph.cpp
 * 
 * Created on: Aug 08, 2018 23:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lattice_planner/lattice_graph.hpp"

#include <iostream>
#include <unordered_map>

using namespace librav;

std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> LatticeGraph::Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter)
{
    int32_t min_h = 4;
    int32_t max_h = 4;

    std::shared_ptr<Graph<LatticeNode, StateLattice>> graph = std::make_shared<Graph<LatticeNode, StateLattice>>();

    auto start_cell = channel->grid_->GetCell(start_index);
    LatticeNode start_node(start_cell, channel);

    std::unordered_map<int32_t, LatticeNode> candidates;
    candidates.insert(std::make_pair(start_node.id, start_node));
    for (int32_t iter = 0; iter < expansion_iter; ++iter)
    {
        std::cout << "candidation size at iteration " << iter << " : " << candidates.size() << std::endl;
        std::unordered_map<int32_t, LatticeNode> added_nodes;
        for (auto &candidate : candidates)
        {
            auto nbs = channel->grid_->GetNeighbours(candidate.second.index.GetX(), candidate.second.index.GetY(), min_h, max_h);
            for (auto nb : nbs)
            {
                LatticeNode next_node(nb, channel);
                StateLattice new_lattice(candidate.second.state, next_node.state);
                if (new_lattice.IsValid())
                {
                    graph->AddEdge(candidate.second, next_node, new_lattice);
                    added_nodes.insert(std::make_pair(next_node.id, next_node));
                }
            }
        }
        candidates = added_nodes;
    }

    return graph;
}
