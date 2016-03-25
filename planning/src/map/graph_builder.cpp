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

Graph<QuadTreeNode>* GraphBuilder::BuildFromQuadTree(QuadTree *tree)
{
	Graph<QuadTreeNode>* graph = new Graph<QuadTreeNode>();

	std::vector<QuadTreeNode*> leaf_nodes;

	// Get all empty leaf nodes
	for(auto it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
	{
		if((*it)->occupancy_ == OccupancyType::FREE)
			leaf_nodes.push_back((*it));
	}

//	std::cout<<"free leaf nodes: "<<leaf_nodes.size()<<std::endl;

	// Find neighbors of each leaf node
	int idx = 0;
	for(auto it = leaf_nodes.begin(); it != leaf_nodes.end(); it++)
	{
		std::vector<QuadTreeNode*> neighbours;

		neighbours = tree->FindNeighbours((*it));
//		std::cout<<"neighbour num: "<<neighbours.size()<<std::endl;

		for(auto itn = neighbours.begin(); itn != neighbours.end(); itn++)
		{
			if((*itn)->occupancy_ == OccupancyType::FREE){
				long x_error =  static_cast<long>((*it)->location_.x) - static_cast<long>((*itn)->location_.x);
				long y_error =  static_cast<long>((*it)->location_.y) - static_cast<long>((*itn)->location_.y);
				double cost = sqrt(static_cast<double>(pow(x_error,2)
						+ pow(y_error,2)));
//				std::cout << "pos a (x,y): ( " <<  (*it)->location_.x << " , "<< (*it)->location_.y << " )"<< std::endl;
//				std::cout << "pos b (x,y): ( " <<  (*itn)->location_.x << " , "<< (*itn)->location_.y << " )" << std::endl;
//				std::cout << "cost: "<< cost << std::endl;
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
			std::vector<SquareCell*> neighbour_list = grid->GetNeighbours(current_nodeid,true);

			for(auto itn = neighbour_list.begin(); itn != neighbour_list.end(); itn++)
			{
				if(grid->cells_[(*itn)->node_id_]->occu_ != OccupancyType::OCCUPIED)
				{
					double error_x,error_y, cost = 0;
					error_x = std::abs(static_cast<long>((*itn)->location_.x) - static_cast<long>((*itc).second->location_.x));
					error_y = std::abs(static_cast<long>((*itn)->location_.y) - static_cast<long>((*itc).second->location_.y));
					cost = std::sqrt(error_x*error_x + error_y*error_y);
//					std::cout << "cost: "<<cost<<std::endl;
					graph->AddEdge((*itc).second, (*itn), cost);

//					graph->AddEdge((*itc).second, (*itn), 1.0);
				}
			}
		}
	}

	return graph;
}

