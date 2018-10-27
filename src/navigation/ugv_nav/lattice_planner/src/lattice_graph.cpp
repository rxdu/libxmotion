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

using namespace librav;

LatticeGraph::LatticeGraph()
{
}

void LatticeGraph::Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter)
{
    int32_t min_h = 6;
    int32_t max_h = 6;

    // auto start_cell = channel->grid_->GetCell(start_index);
    // LatticeNode start_node(start_cell, channel);
    // auto nbs = channel->grid_->GetNeighbours(start_index.GetX(), start_index.GetY(), min_h, max_h);
    // for (auto nb : nbs)
    // {
    //     LatticeNode next_node(nb, channel);
    //     StateLattice new_lattice(start_node.state, next_node.state);
    //     if (new_lattice.IsValid())
    //         AddEdge(start_node, next_node, new_lattice);

    //     // nb->PrintInfo();
    //     // std::cout << next_node << std::endl;
    //     // std::cout << "---" << std::endl;
    // }

    // for (int32_t h = start_index.GetX() + min_h; h <= horizon; h = h + min_h)
    // {
    //     for (int32_t w = -channel->grid_->GetOneSideGridNumber(); w <= channel->grid_->GetOneSideGridNumber(); ++w)
    //     {
    //         auto prev_cell = channel->grid_->GetCell(h, w);
    //         LatticeNode prev_node(prev_cell, channel);

    //         auto nbs = channel->grid_->GetNeighbours(h, w, min_h, max_h);
    //         for (auto nb : nbs)
    //         {
    //             LatticeNode next_node(nb, channel);
    //             StateLattice new_lattice(prev_node.state, next_node.state);
    //             if (new_lattice.IsValid())
    //                 AddEdge(prev_node, next_node, new_lattice);

    //             // nb->PrintInfo();
    //             // std::cout << next_node << std::endl;
    //             // std::cout << "---" << std::endl;
    //         }
    //     }
    // }

    auto start_cell = channel->grid_->GetCell(start_index);
    LatticeNode start_node(start_cell, channel);

    std::vector<LatticeNode> candidates;
    candidates.push_back(start_node);
    for (int32_t iter = 0; iter < expansion_iter; ++iter)
    {
        std::cout << "candidation size at iteration " << iter << " : " << candidates.size() << std::endl;
        std::vector<LatticeNode> added_nodes;
        for (auto &candidate : candidates)
        {
            auto nbs = channel->grid_->GetNeighbours(candidate.index.GetX(), candidate.index.GetY(), min_h, max_h);
            for (auto nb : nbs)
            {
                LatticeNode next_node(nb, channel);
                StateLattice new_lattice(candidate.state, next_node.state);
                if (new_lattice.IsValid())
                {
                    AddEdge(candidate, next_node, new_lattice);
                    added_nodes.push_back(next_node);
                }
            }
        }
        candidates = added_nodes;
    }
}
