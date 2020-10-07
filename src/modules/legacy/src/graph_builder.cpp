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

#include "map/graph_builder.hpp"

using namespace autodrive;

std::shared_ptr<Graph<SquareCell *>> Planner::BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move)
{
	std::shared_ptr<Graph<SquareCell *>> graph = std::make_shared<Graph<SquareCell *>>();

	for (int32_t i = 0; i < grid->SizeX(); ++i)
		for (int32_t j = 0; j < grid->SizeY(); ++j)
		{
			auto cell = grid->GetCell(i,j);
			uint64_t current_nodeid = cell->id;

			if (cell->label == SquareCellLabel::FREE)
			{
				auto neighbour_list = grid->GetNeighbours(current_nodeid, allow_diag_move);

				for (auto &neighbour : neighbour_list)
				{
					if (neighbour->label == SquareCellLabel::FREE)
					{
						double error_x, error_y, cost = 0;
						error_x = std::abs(neighbour->center.x - cell->center.x);
						error_y = std::abs(neighbour->center.y - cell->center.y);
						cost = std::sqrt(error_x * error_x + error_y * error_y);

						graph->AddEdge(cell, neighbour, cost);
					}
				}
			}
		}

	return graph;
}
