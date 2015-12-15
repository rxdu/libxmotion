/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include <map>
#include <cstdint>
#include "graph_types.h"

namespace srcl_ctrl {

class Graph
{
public:
	Graph();
	~Graph();

private:
	std::map<uint64_t, Vertex*> vertex_map_;

private:
	Vertex* GetVertex(const uint64_t vertex_id);

public:
	void AddEdge(uint64_t src_id, uint64_t dst_id, double cost);
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
