/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include <iostream>

#include "graph_builder.h"

using namespace srcl_ctrl;

GraphBuilder::GraphBuilder()
{
}

GraphBuilder::~GraphBuilder()
{

}

Graph<QuadTreeNode>* GraphBuilder::BuildFromQuadTree(QuadTree *tree)
{
	Graph<QuadTreeNode>* graph_ = new Graph<QuadTreeNode>();

	std::vector<QuadTreeNode*> leaf_nodes;

	// Get all empty leaf nodes
	std::vector<QuadTreeNode*>::iterator it;
	for(it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
	{
		if((*it)->occupancy_ == OccupancyType::FREE)
			leaf_nodes.push_back((*it));
	}

//	std::cout<<"free leaf nodes: "<<leaf_nodes.size()<<std::endl;

	// Find neighbors of each leaf node
	for(it = leaf_nodes.begin(); it != leaf_nodes.end(); it++)
	{
		std::vector<QuadTreeNode*> neighbours;
		std::vector<QuadTreeNode*>::iterator itn;

		neighbours = tree->FindNeighbours((*it));

		for(itn = neighbours.begin(); itn != neighbours.end(); itn++)
		{
			if((*itn)->occupancy_ == OccupancyType::FREE)
				graph_->AddEdge((*it), (*itn), 1.0);
		}
	}

//	std::cout<<"graph vertex num: "<<graph_->GetGraphVertices().size()<<std::endl;

	return graph_;
}


