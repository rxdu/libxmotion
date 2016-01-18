/*
 * graph_types.h
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_TYPES_H_
#define SRC_GRAPH_GRAPH_TYPES_H_

#include <vector>
#include <cstdint>

#include "quad_tree.h"

namespace srcl_ctrl {

template<class VertexNodeType>
class Vertex;

template<class EdgeVertexType>
class Edge
{
public:
	Edge(EdgeVertexType *d = nullptr, double c = 0.0):
			dst_(d), cost_(c){};

	EdgeVertexType *dst_;
	double cost_;
};

template<class VertexNodeType>
class Vertex
{
public:
	Vertex(VertexNodeType *node = nullptr):
		node_(node), vertex_id_(node->node_id_){};

	VertexNodeType *node_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex>> adj_;
};

}

#endif /* SRC_GRAPH_GRAPH_TYPES_H_ */
