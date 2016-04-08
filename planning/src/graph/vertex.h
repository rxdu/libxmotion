/*
 * vertex.h
 *
 *  Created on: Feb 1, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_VERTEX_H_
#define SRC_GRAPH_VERTEX_H_

#include <iostream>

namespace srcl_ctrl {

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/
/// An edge data structure template.
template<typename EdgeVertexType>
class Edge
{
public:
	Edge(EdgeVertexType *src = nullptr, EdgeVertexType *dst = nullptr, double c = 0.0):
		src_(src),dst_(dst), cost_(c){};

	EdgeVertexType *src_;
	EdgeVertexType *dst_;
	double cost_;

	bool operator ==(const Edge<EdgeVertexType> other)
	{
		if((src_ == other.src_ && dst_ == other.dst_)
			|| (src_ == other.dst_ && dst_ == other.src_))
			return true;
		else
			return false;
	}

	void PrintEdge()
	{
		std::cout << "Edge: start - " << src_->vertex_id_ << " , end - " << dst_->vertex_id_ << " , cost - " << cost_ << std::endl;
	}
};

/****************************************************************************/
/*								 Vertex										*/
/****************************************************************************/
/// A vertex data structure template.
template<typename VertexNodeType>
class Vertex
{
public:
	Vertex(VertexNodeType *node = nullptr):
		node_(node), vertex_id_(node->node_id_),
		is_checked_(false), is_in_openlist_(false),
		search_parent_(nullptr),
		f_astar_(0),g_astar_(0),h_astar_(0){};
	~Vertex(){
		edges_.clear();
	};

	VertexNodeType *node_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex<VertexNodeType>>> edges_;

	// member variables for search
	bool is_checked_;
	bool is_in_openlist_;
	double f_astar_;
	double g_astar_;
	double h_astar_;
	Vertex<VertexNodeType>* search_parent_;

	bool operator ==(const Vertex<VertexNodeType> other)
	{
		if(vertex_id_ == other.vertex_id_)
			return true;
		else
			return false;
	}

	void ClearVertexSearchInfo(){
		is_checked_ = false;
		is_in_openlist_ = false;
		search_parent_ = nullptr;

		f_astar_ = 0.0;
		g_astar_ = 0.0;
		h_astar_ = 0.0;
	}

	double GetEdgeCost(Vertex<VertexNodeType>* dst_node)
	{
		double cost = -1;
		for(auto ite = edges_.begin(); ite != edges_.end(); ite++)
		{
			if((*ite).dst_->vertex_id_ == dst_node->vertex_id_)
			{
				cost = (*ite).cost_;
				break;
			}
		}

		return cost;
	}
};

}

#endif /* SRC_GRAPH_VERTEX_H_ */
