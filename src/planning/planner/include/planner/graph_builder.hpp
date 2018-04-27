/* 
 * graph_builder.h
 * 
 * Created on: Dec 14, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GRAPH_BUILDER_HPP
#define GRAPH_BUILDER_HPP

#include <memory>

#include "graph/graph.hpp"
#include "decomp/square_grid.hpp"

namespace librav
{
namespace Planner
{
	std::shared_ptr<Graph_t<SquareCell *>> BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move = true);
};
}

#endif /* GRAPH_BUILDER_HPP */
