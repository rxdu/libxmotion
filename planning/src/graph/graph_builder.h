/*
 * graph_builder.h
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_BUILDER_H_
#define SRC_GRAPH_GRAPH_BUILDER_H_

#include "graph.h"
#include "quad_tree.h"

namespace srcl_ctrl {

class GraphBuilder
{
public:
	GraphBuilder();
	~GraphBuilder();

public:
//	Graph<QuadTreeNode>* graph_;

public:
	static Graph<QuadTreeNode>* BuildFromQuadTree(QuadTree* tree);
};
}

#endif /* SRC_GRAPH_GRAPH_BUILDER_H_ */
