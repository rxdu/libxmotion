/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include <map>
#include <vector>
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
	Vertex* GetVertex(const TreeNode* vertex_node);

public:
	void AddEdge(uint64_t src_id, uint64_t dst_id, double cost);
	void AddEdge(TreeNode* src_node, TreeNode* dst_node, double cost);
	std::vector<Vertex*> GetGraphVertices();
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
