/* 
 * graph_builder.cpp
 * 
 * Created on: Dec 14, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include <iostream>
#include <cmath>

#include "planner/graph_builder.hpp"

using namespace librav;

std::shared_ptr<Graph_t<SquareCell *>> Planner::BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move)
{
	std::shared_ptr<Graph_t<SquareCell *>> graph = std::make_shared<Graph_t<SquareCell *>>();

	for (auto &cell_col : grid->grid_cells_)
		for (auto &cell : cell_col)
		{
			uint64_t current_nodeid = cell->id_;

			if (cell->occupancy_ == OccupancyType::FREE)
			{
				auto neighbour_list = grid->GetNeighbours(current_nodeid, allow_diag_move);

				for (auto &neighbour : neighbour_list)
				{
					if (neighbour->occupancy_ == OccupancyType::FREE)
					{
						double error_x, error_y, cost = 0;
						error_x = std::abs(neighbour->position_.x - cell->position_.x);
						error_y = std::abs(neighbour->position_.y - cell->position_.y);
						cost = std::sqrt(error_x * error_x + error_y * error_y);

						graph->AddEdge(cell, neighbour, cost);
					}
				}
			}
		}

	return graph;
}
