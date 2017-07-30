/*
 * map_utils.h
 *
 *  Created on: Apr 26, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_MAP_UTILS_H_
#define SRC_MAP_MAP_UTILS_H_

#include <cstdint>
#include <vector>
#include <string>
#include <memory>

// opencv
#include "opencv2/opencv.hpp"

#include "graph/graph.h"
#include "geometry/square_grid/square_grid.h"
#include "geometry/quadtree/quad_tree.h"
#include "common/planning_types.h"
#include "map/map_info.h"

namespace librav {

class MapUtils{
public:
	MapUtils(){};
	~MapUtils(){};

public:
	static bool ReadImageFromFile(std::string map_path, cv::OutputArray _dst);
	static std::shared_ptr<SquareGrid> CreateSquareGrid(uint32_t row_size, uint32_t col_size, uint32_t cell_size);

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

	static Position2Dd CoordinatesFromMapToMapWorld(Position2D map_pos, MapInfo info);
	static Position2D CoordinatesFromMapWorldToMap(Position2Dd world_pos, MapInfo info);

	static Position2D CoordinatesFromPaddedToOriginal(Position2D pad_pos, MapInfo info);
	static Position2D CoordinatesFromOriginalToPadded(Position2D ori_pos, MapInfo info);

	// ref world frame: x points forward, y points towards left
	static Position2Dd CoordinatesFromMapWorldToRefWorld(Position2Dd map_world_pos, MapInfo info);
	static Position2Dd CoordinatesFromRefWorldToMapWorld(Position2Dd ref_world_pos, MapInfo info);

	static Position2Dd CoordinatesFromMapPaddedToRefWorld(Position2D map_pos, MapInfo info);
	static Position2D CoordinatesFromRefWorldToMapPadded(Position2Dd world_pos, MapInfo info);
};

}

#endif /* SRC_MAP_MAP_UTILS_H_ */
