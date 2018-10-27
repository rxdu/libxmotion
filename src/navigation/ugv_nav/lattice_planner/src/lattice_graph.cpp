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

void LatticeGraph::Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t horizon)
{
    auto start_cell = channel->grid_->GetCell(start_index);
    LatticeNode start_node(start_cell, channel);

    // auto next_node = channel->grid_->GetCell(CurviGridIndex(start_index.GetX() + 1, start_index.GetY() - 1));
    // next_node->PrintInfo();
    // LatticeNode next_lattice_node(next_node, channel);
    // StateLattice new_lattice(start_lattice_node.state, next_lattice_node.state);
    // if (new_lattice.IsValid())
    //     AddEdge(start_lattice_node, next_lattice_node, new_lattice);
    // std::cout << start_lattice_node << std::endl;
    // std::cout << next_lattice_node << std::endl;

    auto nbs = channel->grid_->GetNeighbours(start_index.GetX(), start_index.GetY(), 2, 3);
    std::cout << "size of neighbour: " << nbs.size() << std::endl;
    for (auto nb : nbs)
    {
        LatticeNode next_node(nb, channel);
        StateLattice new_lattice(start_node.state, next_node.state);
        if (new_lattice.IsValid())
            AddEdge(start_node, next_node, new_lattice);

        nb->PrintInfo();
        std::cout << next_node << std::endl;
        std::cout << "---" << std::endl;
    }

    for (int32_t h = start_index.GetX() + 2; h <= horizon; h = h + 2)
    {
        for (int32_t w = -channel->grid_->GetOneSideGridNumber(); w <= channel->grid_->GetOneSideGridNumber(); ++w)
        {
            auto prev_cell = channel->grid_->GetCell(h, w);
            LatticeNode prev_node(prev_cell, channel);

            auto nbs = channel->grid_->GetNeighbours(h, w, 2, 3);
            for (auto nb : nbs)
            {
                LatticeNode next_node(nb, channel);
                StateLattice new_lattice(prev_node.state, next_node.state);
                if (new_lattice.IsValid())
                    AddEdge(prev_node, next_node, new_lattice);

                // nb->PrintInfo();
                // std::cout << next_node << std::endl;
                // std::cout << "---" << std::endl;
            }
        }
    }
}
