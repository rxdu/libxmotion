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

namespace srcl_ctrl {

struct vertex;

typedef struct edge
{
	struct vertex *dst_;
	double cost_;

	edge(struct vertex *d = nullptr, double c = 0.0):
		dst_(d), cost_(c){};
}Edge;

typedef struct vertex
{
	uint64_t vertex_id_;
	std::vector<Edge> adj_;

	vertex(uint64_t id):
		vertex_id_(id){};
}Vertex;

}

#endif /* SRC_GRAPH_GRAPH_TYPES_H_ */
