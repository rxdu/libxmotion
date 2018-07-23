/* 
 * road_grid_traversal.hpp
 * 
 * Created on: Apr 12, 2018 18:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_GRID_TRAVERSAL_HPP
#define ROAD_GRID_TRAVERSAL_HPP

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

#include "traffic_flow/road_square_grid.hpp"

#define MINIMAL_PRINTOUT 1

namespace librav
{

template <typename StateType, typename TransitionType = double>
using GetNeighbourFunc_t = std::function<std::vector<std::tuple<StateType, TransitionType>>(StateType)>;

/// Dijkstra search algorithm.
struct RoadGridTraversal
{
    template <typename StateType, typename TransitionType>
    static void Traverse(StateType start_state, std::string channel, RoadSquareGrid *grid, GetNeighbourFunc_t<StateType, TransitionType> get_neighbours)
    {
        using GraphVertexType = Vertex_t<StateType, TransitionType>;

        // create a new graph with only start and goal vertices
        Graph_t<StateType, TransitionType> graph;
        graph.AddVertex(start_state);

        GraphVertexType *start_vtx = graph.GetVertex(start_state);

        // open list - a list of vertices that need to be checked out
        PriorityQueue<GraphVertexType *> openlist;

        // begin with start vertex
        openlist.put(start_vtx, 0);
        start_vtx->is_in_openlist_ = true;
        start_vtx->g_astar_ = 0;

        // start search iterations
        GraphVertexType *current_vertex;
        while (!openlist.empty())
        {
            current_vertex = openlist.get();
            if (current_vertex->is_checked_)
                continue;

            current_vertex->is_in_openlist_ = false;
            current_vertex->is_checked_ = true;

            std::vector<std::tuple<StateType, TransitionType>> neighbours = get_neighbours(current_vertex->state_);
            for (auto &nb : neighbours)
                graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));

            // check all adjacent vertices (successors of current vertex)
            for (auto edge = current_vertex->edge_begin(); edge != current_vertex->edge_end(); ++edge)
            {
                auto successor = edge->dst_;

                // check if the vertex has been checked (in closed list)
                if (successor->is_checked_ == false)
                {
                    // first set the parent of the adjacent vertex to be the current vertex
                    auto new_cost = current_vertex->g_astar_ + edge->cost_;

                    // if the vertex is not in open list
                    // or if the vertex is in open list but has a higher cost
                    if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
                    {
                        successor->search_parent_ = current_vertex;
                        successor->g_astar_ = new_cost;

                        grid->GetCell(successor->vertex_id_)->extra_attribute.cost_[channel] = new_cost;
                        // std::cout << "vertex id: " << successor->vertex_id_ << " , " << new_cost << std::endl;

                        openlist.put(successor, successor->g_astar_);
                        successor->is_in_openlist_ = true;
                    }
                }
            }
        }
    };
};
}

#endif /* ROAD_GRID_TRAVERSAL_HPP */
