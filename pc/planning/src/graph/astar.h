/*
 * astar.h
 *
 *  Created on: Jan 18, 2016
 *      Author: rdu
 *  Code Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 *  	2. https://oopscenities.net/2012/02/24/c11-stdfunction-and-stdbind/
 */

#ifndef SRC_GRAPH_ASTAR_H_
#define SRC_GRAPH_ASTAR_H_

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

#include "graph/graph.h"
#include "graph/priority_queue.h"

#define MINIMAL_PRINTOUT 1

namespace srcl_ctrl {

template<typename GraphBDSType>
using GetNeighbourBDSFunc_t = std::function<std::vector<std::tuple<GraphBDSType, double>>(GraphBDSType)>;

/// A* search algorithm.
class AStar{

public:

	/// Search using vertices
	template<typename GraphBDSType, typename GraphVertexType>
	static std::vector<GraphVertexType*> Search(Graph<GraphBDSType>& graph, GraphVertexType *start, GraphVertexType *goal)
	{
		// reset last search information
		graph.ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	template<typename GraphBDSType, typename GraphVertexType>
	static std::vector<GraphVertexType*> Search(std::shared_ptr<Graph<GraphBDSType>> graph, GraphVertexType *start, GraphVertexType *goal)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	template<typename GraphBDSType, typename GraphVertexType>
	static std::vector<GraphVertexType*> Search(Graph<GraphBDSType>* graph, GraphVertexType *start, GraphVertexType *goal)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	/// Search using vertex ids
	template<typename GraphBDSType>
	static std::vector<Vertex<GraphBDSType>*> Search(Graph<GraphBDSType>& graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph.ResetGraphVertices();

		auto start = graph.GetVertexFromID(start_id);
		auto goal = graph.GetVertexFromID(goal_id);

		std::vector<Vertex<GraphBDSType>*> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template<typename GraphBDSType>
	static std::vector<Vertex<GraphBDSType>*> Search(std::shared_ptr<Graph<GraphBDSType>> graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		std::vector<Vertex<GraphBDSType>*> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template<typename GraphBDSType>
	static std::vector<Vertex<GraphBDSType>*> Search(Graph<GraphBDSType>* graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		std::vector<Vertex<GraphBDSType>*> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	/// Biased search with consideration on shortcuts
	template<typename GraphBDSType>
	static std::vector<Vertex<GraphBDSType>*> BiasedSearchWithShortcut(Graph<GraphBDSType>& graph, uint64_t start_id, uint64_t goal_id, double max_rewards, double dist_weight = 0.5, double cell_size = 16)
	{
		// reset last search information
		graph.ResetGraphVertices();

		Vertex<GraphBDSType>* start = graph.GetVertexFromID(start_id);
		Vertex<GraphBDSType>* goal = graph.GetVertexFromID(goal_id);

		std::vector<Vertex<GraphBDSType>*> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return BiasedSearch(start, goal, max_rewards, dist_weight, cell_size);
		else
			return empty;
	}

	/// Incremental search
	template<typename GraphBDSType>
	static std::vector<GraphBDSType> IncSearch(GraphBDSType start, GraphBDSType goal, std::function<std::vector<std::tuple<GraphBDSType, double>>(GraphBDSType)> get_neighbour_bds)
	{
		Graph<GraphBDSType> graph;

		bool found_path = false;
		std::vector<Vertex<GraphBDSType>*> path;
		std::vector<GraphBDSType> path_bds;
		Vertex<GraphBDSType>* current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<Vertex<GraphBDSType>*> openlist;

		// first add start and goal node to graph
		Vertex<GraphBDSType>* start_vtx = graph.AddVertex(start);
		Vertex<GraphBDSType>* goal_vtx = graph.AddVertex(goal);

		openlist.put(start_vtx, 0);
		start_vtx->is_in_openlist_ = true;

		//start->search_parent_ = start;
		start_vtx->g_astar_ = 0;

		while(!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if(current_vertex->is_checked_)
				continue;
			if(current_vertex == goal_vtx){
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			std::vector<std::tuple<GraphBDSType, double>> neighbour_bds = get_neighbour_bds(current_vertex->bundled_data_);
			for(auto& nb : neighbour_bds)
				graph.AddEdge(current_vertex->bundled_data_, std::get<0>(nb), std::get<1>(nb));

			std::vector<Edge<Vertex<GraphBDSType>*>> successors = current_vertex->edges_;

			// check all adjacent vertices (successors of current vertex)
			for(auto& suc:successors)
			{
				Vertex<GraphBDSType>* successor;
				successor = suc.dst_;

				// check if the vertex has been checked (in closed list)
				if(successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					double new_cost = current_vertex->g_astar_ + suc.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if(successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
					{
						successor->search_parent_ = current_vertex;
						successor->g_astar_ = new_cost;

						successor->h_astar_ = successor->CalcHeuristic(goal_vtx);
						successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

						openlist.put(successor, successor->f_astar_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		// reconstruct path from search
		if(found_path)
		{
			std::cout << "path found" << std::endl;
			Vertex<GraphBDSType>* waypoint = goal_vtx;
			while(waypoint != start_vtx)
			{
				path.push_back(waypoint);
				waypoint = waypoint->search_parent_;
			}
			// add the start node
			path.push_back(waypoint);
			std::reverse(path.begin(), path.end());

			auto traj_s = path.begin();
			auto traj_e = path.end() - 1;
#ifdef MINIMAL_PRINTOUT
			std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
			std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
			std::cout << "path length: " << path.size() << std::endl;
			std::cout << "total cost: " << path.back()->g_astar_ << std::endl;
#endif

			for(auto& wp : path)
				path_bds.push_back(wp->bundled_data_);
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return path_bds;
	};

private:
	template<typename GraphVertexType>
	static std::vector<GraphVertexType*> Search(GraphVertexType *start, GraphVertexType *goal)
	{
		bool found_path = false;
		std::vector<GraphVertexType*> trajectory;
		GraphVertexType* current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType*> openlist;

		openlist.put(start, 0);
		start->is_in_openlist_ = true;

		//start->search_parent_ = start;
		start->g_astar_ = 0;

		while(!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if(current_vertex->is_checked_)
				continue;
			if(current_vertex == goal){
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			// check all adjacent vertices (successors of current vertex)
			for(auto ite = current_vertex->edges_.begin(); ite != current_vertex->edges_.end(); ite++)
			{
				GraphVertexType* successor;
				successor = (*ite).dst_;

				// check if the vertex has been checked (in closed list)
				if(successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					double new_cost = current_vertex->g_astar_ + (*ite).cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if(successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
					{
						successor->search_parent_ = current_vertex;
						successor->g_astar_ = new_cost;

						successor->h_astar_ = successor->CalcHeuristic(goal);
						successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

						openlist.put(successor, successor->f_astar_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		// reconstruct path from search
		if(found_path)
		{
			std::cout << "path found" << std::endl;
			GraphVertexType* waypoint = goal;
			while(waypoint != start)
			{
				trajectory.push_back(waypoint);
				waypoint = waypoint->search_parent_;
			}
			// add the start node
			trajectory.push_back(waypoint);
			std::reverse(trajectory.begin(), trajectory.end());

			auto traj_s = trajectory.begin();
			auto traj_e = trajectory.end() - 1;
#ifdef MINIMAL_PRINTOUT
			std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
			std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
			std::cout << "path length: " << trajectory.size() << std::endl;
			std::cout << "total cost: " << trajectory.back()->g_astar_ << std::endl;
#endif
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return trajectory;
	};

	template<typename GraphBDSType>
	static Path_t<GraphBDSType> BiasedSearch(Vertex_t<GraphBDSType>* start_vtx, Vertex_t<GraphBDSType>* goal_vtx, double max_rewards, double dist_weight, double cell_size)
	{
		bool found_path = false;
		Path_t<GraphBDSType> path;
		Vertex_t<GraphBDSType>* current_vertex;

		// open list - a list of vertices that need to be checked out
		PriorityQueue<Vertex_t<GraphBDSType>*> openlist;

		openlist.put(start_vtx, 0);
		start_vtx->is_in_openlist_ = true;

		start_vtx->g_astar_ = 0;
		start_vtx->shortcut_rewards_ = 0;
		start_vtx->reward_num_ = 0;
		start_vtx->weighted_cost_ = max_rewards;// 0;

		while(!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if(current_vertex->is_checked_)
				continue;
			if(current_vertex == goal_vtx){
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			// check all adjacent vertices (successors of current vertex)
			for(auto ite = current_vertex->edges_.begin(); ite != current_vertex->edges_.end(); ite++)
			{
				Vertex_t<GraphBDSType>* successor;
				successor = (*ite).dst_;

				// check if the vertex has been checked (in closed list)
				if(successor->is_checked_ == false)
				{
					double new_rewards = current_vertex->shortcut_cost_ + (current_vertex->shortcut_rewards_ - successor->shortcut_rewards_ ) + (1-successor->shortcut_rewards_/max_rewards)*cell_size;
					double new_dist = current_vertex->g_astar_ + (*ite).cost_;

					//double new_cost = current_vertex->weighted_cost_ + (new_dist*dist_weight + new_rewards*(1-dist_weight));
					double new_cost = new_dist*dist_weight + new_rewards*(1-dist_weight);// + avg_rewards; // avg_rewards*(1-dist_weight);

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if(successor->is_in_openlist_ == false || new_cost < successor->weighted_cost_)
					{
						successor->search_parent_ = current_vertex;
						successor->weighted_cost_ = new_cost;

						successor->shortcut_cost_ = new_rewards;
						successor->g_astar_ = new_dist;

						openlist.put(successor, successor->weighted_cost_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		if(found_path)
		{
			std::cout << "path found in nav field" << std::endl;
			Vertex_t<GraphBDSType>* waypoint = goal_vtx;
			while(waypoint != start_vtx)
			{
				path.push_back(waypoint);
				waypoint = waypoint->search_parent_;
			}
			// add the start node
			path.push_back(waypoint);
			std::reverse(path.begin(), path.end());

			auto traj_s = path.begin();
			auto traj_e = path.end() - 1;
#ifndef MINIMAL_PRINTOUT
			std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
			std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
			std::cout << "path length: " << path.size() << std::endl;
			std::cout << "total dist cost: " << path.back()->g_astar_ << std::endl;
			std::cout << "total shortcut cost: " << path.back()->shortcut_cost_ << std::endl;
			std::cout << "total cost with rewards: " << path.back()->weighted_cost_ << std::endl;
#endif
		}
		else
			std::cout << "failed to find a path in nav field" << std::endl;

		return path;
	};
};

}

#endif /* SRC_GRAPH_ASTAR_H_ */
