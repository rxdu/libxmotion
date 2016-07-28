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
#include "map/map_info.h"

namespace srcl_ctrl {

class MapUtils{
public:
	MapUtils(){};
	~MapUtils(){};

public:
	static std::vector<Position2D> GetWaypointsFromSGridPath(std::vector<Vertex<SquareCell*>*>& path);
	static std::vector<Position2D> GetWaypointsFromQTreePath(std::vector<Vertex<QuadTreeNode*>*>& path);

	template<typename T>
	static std::vector<Position2D> GetWaypointsFromVertexSequence(std::vector<Vertex_t<T>*>& path)
	{
		std::vector<Position2D> waypoints;

		for(auto& vt: path)
			waypoints.push_back(vt->bundled_data_->location_);

		return waypoints;
	}

	static Position2Dd CoordinatesFromMapToWorld(Position2D map_pos, MapInfo info);
	static Position2D CoordinatesFromWorldToMap(Position2Dd world_pos, MapInfo info);
	static Position2D CoordinatesFromPaddedToOriginal(Position2D pad_pos, MapInfo info);
	static Position2D CoordinatesFromOriginalToPadded(Position2D ori_pos, MapInfo info);
};

}

#endif /* SRC_MAP_MAP_UTILS_H_ */
