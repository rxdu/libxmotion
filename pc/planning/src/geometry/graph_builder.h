/*
 * graph_builder.h
 *
 *  Created on: Dec 14, 2015
 *      Author: rdu
 */

#ifndef SRC_GRAPH_GRAPH_BUILDER_H_
#define SRC_GRAPH_GRAPH_BUILDER_H_

#include <memory>

#include "graph/graph.h"
#include "quadtree/quad_tree.h"
#include "square_grid/square_grid.h"
#include "cube_array/cube_array.h"

namespace srcl_ctrl {

class GraphBuilder
{
public:
	GraphBuilder();
	~GraphBuilder();

public:
	static std::shared_ptr<Graph<QuadTreeNode*>> BuildFromQuadTree(const std::shared_ptr<QuadTree>& tree);
	static std::shared_ptr<Graph<SquareCell*>> BuildFromSquareGrid(const std::shared_ptr<SquareGrid>& grid, bool allow_diag_move);

	static std::shared_ptr<Graph<CubeCell&>> BuildFromCubeArray(const std::shared_ptr<CubeArray>& cube_array);
};
}

#endif /* SRC_GRAPH_GRAPH_BUILDER_H_ */
