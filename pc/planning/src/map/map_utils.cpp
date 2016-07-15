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

