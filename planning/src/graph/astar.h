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
	std::vector<GraphVertexType> Search(GraphVertexType *start, GraphVertexType *goal){
		bool found_path = false;
		std::vector<GraphVertexType> trajectory;
		GraphVertexType current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType> openlist;

		openlist.put(start, 0);
		start->is_under_checking = true;

		start->search_parent_ = start;
		start->search_cost_ = 0;

		while(!openlist.empty())
		{
			current_vertex = openlist.get();
			current_vertex.is_under_checking = false;
			current_vertex.is_checked_ = true;

			if(current_vertex == goal){
				found_path = true;
				break;
			}

			// find all adjacent vertices
//			std::vector<GraphVertexType*> adjacent_vertices;
			typename std::vector<Edge<GraphVertexType>>::iterator ite;
			for(ite = current_vertex.adj_.begin(); ite != current_vertex.adj_.end(); ite++)
			{
//				adjacent_vertices.push_back((*ite)->dst_);
				if((*ite)->dst_.is_checked_ == false)
				{
					// check if the vertex is already in open list
					if((*ite)->dst_.is_under_checking == false)
					{
						(*ite)->dst_.search_parent_ = current_vertex;
						double cost_so_far = current_vertex.search_cost_so_far_ +
								(*ite).cost_ +
								CalcHeuristic((*ite)->dst_, goal);
						openlist.put((*ite)->dst_, cost_so_far);
					}
					else
					{
						double cost1 = current_vertex.search_parent_.GetEdgeCost(current_vertex) + (*ite).cost_;
						double cost2 = current_vertex.search_parent_.GetEdgeCost((*ite)->dst_);

						if(cost1 < cost2) {
							current_vertex = current_vertex.search_parent_;
							double cost_so_far = current_vertex.search_cost_so_far_ +
									cost2 +
									CalcHeuristic((*ite)->dst_, goal);
							openlist.put((*ite)->dst_, cost_so_far);
						}
					}
				}
			}
		}

		// reconstruct path from search
		if(found_path)
		{

		}


		return trajectory;
	};

private:
	double CalcHeuristic(GraphVertexType vertex_a, GraphVertexType vertex_b)
	{
		double x1,x2,y1,y2;

		x1 = vertex_a.node_.location_.x;
		y1 = vertex_a.node_.location_.y;

		x2 = vertex_b.node_.location_.x;
		y2 = vertex_b.node_.location_.y;

		return std::abs(x1-x2) + std::abs(y1-y2);
	};

	void ReconstructPath();
};

}

#endif /* SRC_GRAPH_ASTAR_H_ */
