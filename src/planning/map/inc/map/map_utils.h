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

#include "planning/graph/graph.h"
#include "geometry/square_grid.h"
#include "geometry/quad_tree.h"
#include "common/librav_types.h"
#include "map/map_info.h"

namespace librav {

namespace MapUtils{	
	bool ReadImageFromFile(std::string map_path, cv::OutputArray _dst);
	std::shared_ptr<SquareGrid> CreateSquareGrid(uint32_t row_size, uint32_t col_size, uint32_t cell_size);

	std::vector<Position2Di> GetWaypointsFromSGridPath(std::vector<Vertex<SquareCell*>*>& path);
	std::vector<Position2Di> GetWaypointsFromQTreePath(std::vector<Vertex<QuadTreeNode*>*>& path);

	template<typename T>
	std::vector<Position2Di> GetWaypointsFromVertexSequence(std::vector<Vertex_t<T>*>& path)
	{
		std::vector<Position2Di> waypoints;

		for(auto& vt: path)
			waypoints.push_back(vt->bundled_data_->location_);

		return waypoints;
	}

	Position2Dd CoordinatesFromMapToMapWorld(Position2Di map_pos, MapInfo info);
	Position2Di CoordinatesFromMapWorldToMap(Position2Dd world_pos, MapInfo info);

	Position2Di CoordinatesFromPaddedToOriginal(Position2Di pad_pos, MapInfo info);
	Position2Di CoordinatesFromOriginalToPadded(Position2Di ori_pos, MapInfo info);

	// ref world frame: x points forward, y points towards left
	Position2Dd CoordinatesFromMapWorldToRefWorld(Position2Dd map_world_pos, MapInfo info);
	Position2Dd CoordinatesFromRefWorldToMapWorld(Position2Dd ref_world_pos, MapInfo info);

	Position2Dd CoordinatesFromMapPaddedToRefWorld(Position2Di map_pos, MapInfo info);
	Position2Di CoordinatesFromRefWorldToMapPadded(Position2Dd world_pos, MapInfo info);
}

}

#endif /* SRC_MAP_MAP_UTILS_H_ */
