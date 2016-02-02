/*
 * astar.h
 *
 *  Created on: Jan 18, 2016
 *      Author: rdu
 *  Code Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 */

#ifndef SRC_GRAPH_ASTAR_H_
#define SRC_GRAPH_ASTAR_H_

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>

#include <vertex.h>

namespace srcl_ctrl {

template<typename T, typename Number=int>
struct PriorityQueue {
	typedef std::pair<Number, T> PQElement;

	std::priority_queue<PQElement, std::vector<PQElement>,
	std::greater<PQElement>> elements;

	inline bool empty() const { return elements.empty(); }

	inline void put(T item, Number priority) {
		elements.emplace(priority, item);
	}

	inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

template<typename GraphVertexType>
class AStar{
public:
	AStar(){};
	~AStar(){};

public:
	std::vector<GraphVertexType*> Search(GraphVertexType *start, GraphVertexType *goal){
		bool found_path = false;
		std::vector<GraphVertexType*> trajectory;
		GraphVertexType* current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType*> openlist;

		openlist.put(start, 0);
		start->is_in_openlist_ = true;

		start->search_parent_ = start;
		start->g_astar_ = 0;

		while(!openlist.empty())
		{
			current_vertex = openlist.get();
			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			if(current_vertex == goal){
				found_path = true;
				break;
			}

			// check all adjacent vertices (successors of current vertex)
			typename std::vector<Edge<GraphVertexType>>::iterator ite;
			for(ite = current_vertex->adj_.begin(); ite != current_vertex->adj_.end(); ite++)
			{
				GraphVertexType* successor;
				successor = (*ite).dst_;

				// check if the vertex has been checked (in closed list)
				if(successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					successor->search_parent_ = current_vertex;

					// if the vertex is not in open list
					if(successor->is_in_openlist_ == false)
					{
						successor->g_astar_ = successor->search_parent_->g_astar_ + (*ite).cost_;
						successor->h_astar_ = CalcHeuristic(successor, goal);
						successor->f_astar_ = successor->g_astar_ + successor->f_astar_;

						openlist.put((*ite).dst_, successor->f_astar_);
					}
					// if the vertex has already been added into open list
					else
					{
						// cost = parent vertex -> current vertex
						//		+ current vertex -> successor vertex
						double cost1 = current_vertex->search_parent_->GetEdgeCost(current_vertex) + (*ite).cost_;
						// cost = parent vertex -> successor vertex
						double cost2 = current_vertex->search_parent_->GetEdgeCost(successor);

						if(cost1 < cost2) {
							current_vertex = current_vertex->search_parent_;
							successor->search_parent_ = current_vertex;

							successor->g_astar_ = successor->search_parent_->g_astar_ +
									current_vertex->search_parent_->GetEdgeCost(successor);
							successor->h_astar_ = CalcHeuristic(successor, goal);
							successor->f_astar_ = successor->g_astar_ + successor->f_astar_;

//							openlist.put((*ite)->dst_, successor->f_astar_);
						}
						else
						{
							successor->g_astar_ = successor->search_parent_->g_astar_ +
									current_vertex->search_parent_->GetEdgeCost(current_vertex) + (*ite).cost_;
							successor->h_astar_ = CalcHeuristic(successor, goal);
							successor->f_astar_ = successor->g_astar_ + successor->f_astar_;

//							openlist.put((*ite)->dst_, successor->f_astar_);
						}
					}
				}
			}
		}

		// reconstruct path from search
		if(found_path)
		{
			std::cout << "path found" << std::endl;
			while(current_vertex != start)
			{
				trajectory.push_back(current_vertex);
				current_vertex = current_vertex->search_parent_;
			}
		}
		else
			std::cout << "failed to find a path" << std::endl;


		return trajectory;
	};

private:
	double CalcHeuristic(GraphVertexType* vertex_a, GraphVertexType* vertex_b)
	{
		double x1,x2,y1,y2;

		x1 = vertex_a->node_->location_.x;
		y1 = vertex_a->node_->location_.y;

		x2 = vertex_b->node_->location_.x;
		y2 = vertex_b->node_->location_.y;

		return std::abs(x1-x2) + std::abs(y1-y2);
	};
};

}

#endif /* SRC_GRAPH_ASTAR_H_ */
