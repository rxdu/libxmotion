/*
 * graph_builder.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#include <iostream>
#include <cmath>

#include "geometry/graph_builder.h"

using namespace srcl_ctrl;

GraphBuilder::GraphBuilder()
{
}

GraphBuilder::~GraphBuilder()
{

}

std::shared_ptr<Graph<QuadTreeNode*>> GraphBuilder::BuildFromQuadTree(const std::shared_ptr<QuadTree>& tree)
{
	std::shared_ptr<Graph<QuadTreeNode*>> graph = std::make_shared<Graph<QuadTreeNode*>>();

	std::vector<QuadTreeNode*> leaf_nodes;

	// Get all empty leaf nodes
	for(auto it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
	{
		if((*it)->occupancy_ == OccupancyType::FREE)
			leaf_nodes.push_back((*it));
	}

	// Find neighbors of each leaf node
	int idx = 0;
	for(auto it = leaf_nodes.begin(); it != leaf_nodes.end(); it++)
	{
		std::vector<QuadTreeNode*> neighbours;

		neighbours = tree->FindNeighbours((*it));

		for(auto itn = neighbours.begin(); itn != neighbours.end(); itn++)
		{
			if((*itn)->occupancy_ == OccupancyType::FREE){
				long x_error =  static_cast<long>((*it)->location_.x) - static_cast<long>((*itn)->location_.x);
				long y_error =  static_cast<long>((*it)->location_.y) - static_cast<long>((*itn)->location_.y);
				double cost = sqrt(static_cast<double>(pow(x_error,2)
						+ pow(y_error,2)));

				graph->AddEdge((*it), (*itn), cost);
			}
		}
	}

	return graph;
}

std::shared_ptr<Graph<SquareCell*>> GraphBuilder::BuildFromSquareGrid(const std::shared_ptr<SquareGrid>& grid, bool allow_diag_move)
{
	std::shared_ptr<Graph<SquareCell*>> graph = std::make_shared<Graph<SquareCell*>>();

	for(auto itc = grid->cells_.begin(); itc != grid->cells_.end(); itc++)
	{
		uint64_t current_nodeid = (*itc).second->data_id_;

		if(grid->cells_[current_nodeid]->occu_ != OccupancyType::OCCUPIED) {
			std::vector<SquareCell*> neighbour_list = grid->GetNeighbours(current_nodeid,allow_diag_move);

			for(auto itn = neighbour_list.begin(); itn != neighbour_list.end(); itn++)
			{
				if(grid->cells_[(*itn)->data_id_]->occu_ != OccupancyType::OCCUPIED)
				{
					double error_x,error_y, cost = 0;
					error_x = std::abs(static_cast<long>((*itn)->location_.x) - static_cast<long>((*itc).second->location_.x));
					error_y = std::abs(static_cast<long>((*itn)->location_.y) - static_cast<long>((*itc).second->location_.y));
					cost = std::sqrt(error_x*error_x + error_y*error_y);

					graph->AddEdge((*itc).second, *itn, cost);
				}
			}
		}
	}

	return graph;
}

std::shared_ptr<Graph<const CubeCell&>> GraphBuilder::BuildFromCubeArray(const std::shared_ptr<CubeArray>& cube_array)
{
	std::shared_ptr<Graph<const CubeCell&>> graph = std::make_shared<Graph<const CubeCell&>>();

	double size = cube_array->cube_size_;

	for(auto& cube:cube_array->cubes_)
	{
		uint64_t current_nodeid = cube.data_id_;

		if(cube_array->cubes_[current_nodeid].occu_ != OccupancyType::OCCUPIED) {
			std::vector<uint64_t> neighbour_list = cube_array->GetNeighbours(current_nodeid);

			for(auto& nid : neighbour_list)
			{
				if(cube_array->cubes_[nid].occu_ != OccupancyType::OCCUPIED)
				{
					graph->AddEdge(cube_array->cubes_[current_nodeid], cube_array->cubes_[nid], size);
				}
			}
		}
	}

	return graph;
}

