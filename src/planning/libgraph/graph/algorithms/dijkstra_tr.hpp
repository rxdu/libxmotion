/* 
 * dijkstra_traversal.hpp
 * 
 * Created on: Mar 30, 2018 15:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DIJKSTRA_TR_HPP
#define DIJKSTRA_TR_HPP

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

#define MINIMAL_PRINTOUT 1

namespace librav
{

template <typename StateType, typename TransitionType = double>
using GetNeighbourFunc_t = std::function<std::vector<std::tuple<StateType, TransitionType>>(StateType)>;

/// Dijkstra search algorithm.
class DijkstraTraversal
{
  public:
    /// Search using vertex ids
    template <typename StateType, typename TransitionType>
    void Search(Graph_t<StateType, TransitionType> *graph, uint64_t start_id, uint64_t goal_id)
    {
        // reset last search information
        graph->ResetGraphVertices();

        auto start = graph->GetVertexFromID(start_id);
        auto goal = graph->GetVertexFromID(goal_id);

        // start a new search and return result
        if (start != nullptr && goal != nullptr)
            Search(start, goal);
    }

    template <typename StateType, typename TransitionType>
    static Graph_t<StateType, TransitionType> IncSearch(std::vector<StateType> start_states, GetNeighbourFunc_t<StateType, TransitionType> get_neighbours)
    {
    }

    template <typename StateType, typename TransitionType>
    static Graph_t<StateType, TransitionType> IncSearch(StateType start_state, GetNeighbourFunc_t<StateType, TransitionType> get_neighbours)
    {
        using GraphVertexType = Vertex_t<StateType, TransitionType>;

        // create a new graph with only start and goal vertices
        Graph_t<StateType, TransitionType> graph;
        graph.AddVertex(start_state);
        graph.AddVertex(goal_state);

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
            for (auto &edge : current_vertex->edges_to_)
            {
                auto successor = edge.dst_;

                // check if the vertex has been checked (in closed list)
                if (successor->is_checked_ == false)
                {
                    // first set the parent of the adjacent vertex to be the current vertex
                    auto new_cost = current_vertex->g_astar_ + edge.cost_;

                    // if the vertex is not in open list
                    // or if the vertex is in open list but has a higher cost
                    if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
                    {
                        successor->search_parent_ = current_vertex;
                        successor->g_astar_ = new_cost;

                        openlist.put(successor, successor->g_astar_);
                        successor->is_in_openlist_ = true;
                    }
                }
            }
        }

        return graph;
    };

  private:
    template <typename StateType, typename TransitionType>
    static Path_t<StateType> Search(Vertex_t<StateType, TransitionType> *start_vtx)
    {
        using GraphVertexType = Vertex_t<StateType, TransitionType>;

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

            // check all adjacent vertices (successors of current vertex)
            for (auto &edge : current_vertex->edges_to_)
            {
                auto successor = edge.dst_;

                // check if the vertex has been checked (in closed list)
                if (successor->is_checked_ == false)
                {
                    // first set the parent of the adjacent vertex to be the current vertex
                    auto new_cost = current_vertex->g_astar_ + edge.cost_;

                    // if the vertex is not in open list
                    // or if the vertex is in open list but has a higher cost
                    if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
                    {
                        successor->search_parent_ = current_vertex;
                        successor->g_astar_ = new_cost;

                        openlist.put(successor, successor->g_astar_);
                        successor->is_in_openlist_ = true;
                    }
                }
            }
        }
    };
};
}

#endif /* DIJKSTRA_TR_HPP */
