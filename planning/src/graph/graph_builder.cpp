/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include "graph_builder.h"

using namespace srcl_ctrl;

GraphBuilder::GraphBuilder():
		graph_(nullptr)
{

}

GraphBuilder::~GraphBuilder()
{

}

Graph* GraphBuilder::BuildFromQuadTree(QuadTree *tree)
{
	graph_ = new Graph();

	std::vector<TreeNode*> leaf_nodes;
	leaf_nodes = tree->leaf_nodes_;

	// Find neighbors of each leaf node
	std::vector<TreeNode*>::iterator it;
	for(it = leaf_nodes.begin(); it != leaf_nodes.end(); it++)
	{
		std::vector<TreeNode*> neighbours;
		std::vector<TreeNode*>::iterator itn;

		neighbours = tree->FindNeighbours((*it));

		for(itn = neighbours.begin(); itn != neighbours.end(); itn++)
		{
			graph_->AddEdge((*it)->node_id_, (*itn)->node_id_, 1.0);
		}
	}

	return graph_;
}


