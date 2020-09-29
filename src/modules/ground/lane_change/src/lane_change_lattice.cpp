/* 
 * lattice_graph.cpp
 * 
 * Created on: Aug 08, 2018 23:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lane_change/lane_change_lattice.hpp"

#include <iostream>
#include <unordered_map>

// #include "local_planner/lattice_dijkstra.hpp"

using namespace librav;

std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> LatticeGraph::Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter, std::vector<int32_t> &final_nodes)
{
    std::shared_ptr<Graph<LatticeNode, StateLattice>> graph = std::make_shared<Graph<LatticeNode, StateLattice>>();

    auto start_cell = channel->grid_->GetCell(start_index);
    LatticeNode start_node(start_cell, channel);

    // TODO: parallelize the graph constrution process (spiral solving)
    std::unordered_map<int32_t, LatticeNode> candidates;
    candidates.insert(std::make_pair(start_node.id, start_node));
    for (int32_t iter = 0; iter < expansion_iter; ++iter)
    {
        // std::cout << "candidation size at iteration " << iter << " : " << candidates.size() << std::endl;
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

        if (added_nodes.empty())
            break;

        candidates = added_nodes;
    }

    final_nodes.clear();
    for (auto &entry : candidates)
        final_nodes.push_back(entry.second.id);

    return graph;
}

std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> LatticeGraph::Construct(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter)
{
    std::vector<int32_t> final_nodes;
    return Construct(channel, start_index, expansion_iter, final_nodes);
}

// std::vector<StateLattice> LatticeGraph::Search(std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter)
// {
//     std::vector<int32_t> final_nodes;
//     auto graph = Construct(channel, start_index, expansion_iter, final_nodes);

//     auto start_cell = channel->grid_->GetCell(start_index);

//     return LatticeDijkstra::Search(graph.get(), start_cell->id, final_nodes);
// }

// std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> LatticeGraph::Search(std::vector<StateLattice> &path, std::shared_ptr<TrafficChannel> channel, CurviGridIndex start_index, int32_t expansion_iter)
// {
//     std::vector<int32_t> final_nodes;
//     auto graph = Construct(channel, start_index, expansion_iter, final_nodes);

//     auto start_cell = channel->grid_->GetCell(start_index);
//     path = LatticeDijkstra::Search(graph.get(), start_cell->id, final_nodes);

//     return graph;
// }

// std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> LatticeGraph::Search(std::vector<StateLattice> &path, std::shared_ptr<TrafficChannel> channel, Pose2d pose, int32_t expansion_iter)
// {
//     auto start_index = channel->GetIndexFromPosition({pose.position.x, pose.position.y});

//     std::cout << "starting index: " << start_index << std::endl;

//     std::vector<int32_t> final_nodes;
//     auto graph = Construct(channel, start_index, expansion_iter, final_nodes);

//     auto start_cell = channel->grid_->GetCell(start_index);
//     path = LatticeDijkstra::Search(graph.get(), start_cell->id, final_nodes);

//     return graph;
// }