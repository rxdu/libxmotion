/*
 * graph_types.h
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_TYPES_H_
#define SRC_GRAPH_GRAPH_TYPES_H_

#include <vector>

namespace srcl {

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
	std::vector<Edge> adj;
}Vertex;

}

#endif /* SRC_GRAPH_GRAPH_TYPES_H_ */
