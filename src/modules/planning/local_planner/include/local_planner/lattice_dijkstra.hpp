/* 
 * lattice dijkstra.hpp
 * 
 * Created on: Nov 30, 2017 14:22
 * Description: Dijkstra's search algorithm with lattice graph
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef LATTICE_DIJKSTRA_HPP
#define LATTICE_DIJKSTRA_HPP

#include <vector>
#include <tuple>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <iostream>
#include <memory>

#include "graph/graph.hpp"
#include "graph/details/priority_queue.hpp"

#include "state_lattice/state_lattice.hpp"
#include "local_planner/lattice_graph.hpp"

namespace autodrive
{
/// Dijkstra search algorithm.
class LatticeDijkstra
{
    using GraphType = Graph<LatticeGraph::LatticeNode, StateLattice>;
    using VertexType = Graph<LatticeGraph::LatticeNode, StateLattice>::Vertex;
    using PathType = std::vector<StateLattice>;

  public:
    static PathType Search(GraphType *graph, int32_t start, int32_t goal)
    {
        // reset last search information
        graph->ResetAllVertices();

        auto start_it = graph->FindVertex(start);

        std::vector<GraphType::vertex_iterator> goal_its;
        auto goal_it = graph->FindVertex(goal);
        if (goal_it != graph->vertex_end())
            goal_its.push_back(goal_it);
        std::cout << "size of goal: " << goal_its.size() << std::endl;

        PathType empty;

        // start a new search and return result
        if (start_it != graph->vertex_end() && !goal_its.empty())
            return PerformSearch(graph, start_it, goal_its);
        else
            return empty;
    }

    static PathType Search(GraphType *graph, int32_t start, std::vector<int32_t> goals)
    {
        // reset last search information
        graph->ResetAllVertices();

        auto start_it = graph->FindVertex(start);

        std::vector<GraphType::vertex_iterator> goal_its;
        for (auto &goal : goals)
        {
            auto goal_it = graph->FindVertex(goal);
            if (goal_it != graph->vertex_end())
                goal_its.push_back(goal_it);
        }
        std::cout << "size of goal: " << goal_its.size() << std::endl;

        PathType empty;

        // start a new search and return result
        if (start_it != graph->vertex_end() && !goal_its.empty())
            return PerformSearch(graph, start_it, goal_its);
        else
            return empty;
    }

  public:
    static PathType PerformSearch(GraphType *graph,
                                  GraphType::vertex_iterator start_vtx,
                                  std::vector<GraphType::vertex_iterator> goal_vtxs)
    {
        using VertexIterator = typename GraphType::vertex_iterator;

        typename GraphType::vertex_iterator goal_vtx;

        LatticeGraph::LatticeNode virtual_node(std::numeric_limits<int64_t>::max());

        for (auto &goal : goal_vtxs)
            graph->AddEdge(goal->state_, virtual_node, StateLattice());
        goal_vtx = graph->FindVertex(virtual_node.id);

        // open list - a list of vertices that need to be checked out
        PriorityQueue<VertexIterator> openlist;

        // begin with start vertex
        openlist.put(start_vtx, 0);
        start_vtx->is_in_openlist_ = true;
        start_vtx->g_cost_ = 0;

        // start search iterations
        bool found_path = false;
        VertexIterator current_vertex;
        while (!openlist.empty() && found_path != true)
        {
            current_vertex = openlist.get();
            if (current_vertex->is_checked_)
                continue;

            if (current_vertex == goal_vtx)
            {
                found_path = true;
                break;
            }

            current_vertex->is_in_openlist_ = false;
            current_vertex->is_checked_ = true;

            // check all adjacent vertices (successors of current vertex)
            for (auto &edge : current_vertex->edges_to_)
            {
                auto successor = edge.dst_;

                // check if the vertex has been checked (in closed list)
                if (successor->is_checked_ == false)
                {
                    // first set the parent of the adjacent vertex to be the current vertex
                    auto new_cost = current_vertex->g_cost_ + edge.cost_.GetLength();

                    // if the vertex is not in open list
                    // or if the vertex is in open list but has a higher cost
                    if (successor->is_in_openlist_ == false || new_cost < successor->g_cost_)
                    {
                        successor->search_parent_ = current_vertex;
                        successor->g_cost_ = new_cost;

                        openlist.put(successor, successor->g_cost_);
                        successor->is_in_openlist_ = true;
                    }
                }
            }
        }

        // reconstruct path from search
        if (found_path)
        {
            std::cout << "path found with cost " << goal_vtx->g_cost_ << std::endl;
            return ReconstructPath(graph, start_vtx, goal_vtx);
        }
        else
            std::cout << "failed to find a path" << std::endl;

        PathType empty_path;

        return empty_path;
    };

    static PathType ReconstructPath(GraphType *graph, GraphType::vertex_iterator start_vtx, GraphType::vertex_iterator goal_vtx)
    {
        PathType path;
        GraphType::vertex_iterator waypoint = goal_vtx->search_parent_;
        while (waypoint != start_vtx)
        {
            path.push_back(waypoint->search_parent_->FindEdge(waypoint->state_)->cost_);
            waypoint = waypoint->search_parent_;
        }
        std::reverse(path.begin(), path.end());

        return path;
    }
};
} // namespace autodrive

#endif /* LATTICE_DIJKSTRA_HPP */
