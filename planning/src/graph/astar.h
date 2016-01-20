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
	std::vector<GraphVertexType> Search(GraphVertexType start, GraphVertexType goal){
		std::vector<GraphVertexType> trajectory;
		GraphVertexType current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType> openlist;

		openlist.put(start, 0);

		while(!openlist.empty())
		{
			current_vertex = openlist.get();

			if(current_vertex == goal)
				break;
			else
				current_vertex.is_checked_ = true;

			// find all adjacent vertices
			std::vector<GraphVertexType*> adjacent_vertices;
			typename std::vector<Edge<GraphVertexType>>::iterator ite;
			for(ite = current_vertex.adj_.begin(); ite != current_vertex.adj_.end(); ite++)
			{
				adjacent_vertices.push_back((*ite)->dst_);
			}
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
