/*
 * map_utils.cpp
 *
 *  Created on: Apr 26, 2016
 *      Author: rdu
 */

#include <map/map_utils.h>

using namespace srcl_ctrl;

MapUtils::MapUtils()
{

}

MapUtils::~MapUtils()
{

}

std::vector<Position2D> MapUtils::GetWaypointsFromSGridPath(std::vector<Vertex<SquareCell*>*>& path)
{
	std::vector<Position2D> waypoints;

	for(auto& vt: path)
		waypoints.push_back(vt->bundled_data_->location_);

	return waypoints;
}

std::vector<Position2D> MapUtils::GetWaypointsFromQTreePath(std::vector<Vertex<QuadTreeNode*>*>& path)
{
	std::vector<Position2D> waypoints;

	for(auto& vt: path)
		waypoints.push_back(vt->bundled_data_->location_);

	return waypoints;
}

Position2Dd MapUtils::CoordinatesFromMapToWorld(Position2D map_pos, MapInfo info)
{
	Position2Dd rpos;

	rpos.x = map_pos.x / info.scale_x;
	rpos.y = map_pos.y / info.scale_y;

	if(rpos.x > info.world_size_x)
		rpos.x = info.world_size_x;
	if(rpos.y > info.world_size_y)
		rpos.y = info.world_size_y;

	std::cout << std::endl;
	std::cout << "map scale: " << info.scale_x << " , " << info.scale_y << std::endl;
	std::cout << "input: " << map_pos.x << " , " << map_pos.y << std::endl;
	std::cout << "convertion result: " << rpos.x << " , " << rpos.y << std::endl;

	return rpos;
}

Position2D MapUtils::CoordinatesFromWorldToMap(Position2Dd map_pos, MapInfo info)
{
	Position2D rpos;

	return rpos;
}

Position2D MapUtils::CoordinatesFromPaddedToOriginal(Position2D pad_pos, MapInfo info)
{
	Position2D rpos;

	return rpos;
}

Position2D MapUtils::CoordinatesFromOriginalToPadded(Position2D ori_pos, MapInfo info)
{
	Position2D rpos;

	return rpos;
}
