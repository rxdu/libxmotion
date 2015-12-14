/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include "graph_types.h"

namespace srcl_ctrl {

class Graph
{
public:
	Graph();
	~Graph();

public:
	void AddEdge();
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
