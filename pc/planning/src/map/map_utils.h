/*
 * map_utils.h
 *
 *  Created on: Apr 26, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_MAP_UTILS_H_
#define SRC_MAP_MAP_UTILS_H_

#include <vector>

#include "graph/graph.h"
#include "square_grid/square_grid.h"
#include "quadtree/quad_tree.h"
#include "common/planning_types.h"

namespace srcl_ctrl {

class MapUtils{
public:
	MapUtils();
	~MapUtils();

public:
	std::vector<Position2D> GetWaypointsFromSGridPath(std::vector<Vertex<SquareCell*>*>& path);
	std::vector<Position2D> GetWaypointsFromQTreePath(std::vector<Vertex<QuadTreeNode*>*>& path);
};

}

#endif /* SRC_MAP_MAP_UTILS_H_ */
