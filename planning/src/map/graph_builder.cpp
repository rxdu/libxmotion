/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include <iostream>
#include <cmath>

#include "graph_builder.h"

using namespace srcl_ctrl;

GraphBuilder::GraphBuilder()
{
}

GraphBuilder::~GraphBuilder()
{

}

// TODO fix the memory leak issue
Graph<QuadTreeNode>* GraphBuilder::BuildFromQuadTree(QuadTree *tree)
{
	Graph<QuadTreeNode>* graph = new Graph<QuadTreeNode>();

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
			if((*itn)->occupancy_ == OccupancyType::FREE){
				double cost = sqrt(pow((double)((*it)->location_.x - (*itn)->location_.x),2)
						+ pow((double)((*it)->location_.y - (*itn)->location_.y),2));
				graph->AddEdge((*it), (*itn), cost);
			}
		}
	}

//	std::cout<<"graph vertex num: "<<graph_->GetGraphVertices().size()<<std::endl;

	return graph;
}

Graph<SquareCell>* GraphBuilder::BuildFromSquareGrid(SquareGrid* grid)
{
	Graph<SquareCell>* graph = new Graph<SquareCell>();

	for(auto itc = grid->cells_.begin(); itc != grid->cells_.end(); itc++)
	{
		uint64_t current_nodeid = (*itc).second->node_id_;

		if(grid->cells_[current_nodeid]->occu_ != OccupancyType::OCCUPIED) {
			std::vector<SquareCell*> neighbour_list = grid->GetNeighbours(current_nodeid);

			for(auto itn = neighbour_list.begin(); itn != neighbour_list.end(); itn++)
			{
				if(grid->cells_[(*itn)->node_id_]->occu_ != OccupancyType::OCCUPIED)
					graph->AddEdge((*itc).second, (*itn), 1.0);
			}
		}
	}

	return graph;
}

