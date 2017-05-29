/*
 * state_vadility_checker_2d.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_
#define PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_

#include "opencv2/opencv.hpp"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "map/map_info.h"
#include "map/image_utils.h"
#include "map/map_utils.h"

namespace srcl_ctrl {

class StateValidityChecker2D : public ompl::base::StateValidityChecker {
public:
	StateValidityChecker2D(const ompl::base::SpaceInformationPtr &si) :
       ompl::base::StateValidityChecker(si), occupancy_map_ready_(false) {}

public:
	cv::Mat occupancy_map_;
	MapInfo map_info_;
	bool occupancy_map_ready_;

public:
	void SetOccupancyMap(cv::Mat _src, MapInfo info)
	{
		//std::cout << "map size: " << _src.size().height << " , " << _src.size().width << std::endl;
		occupancy_map_.create(_src.size(), CV_8UC1);
		_src.copyTo(occupancy_map_);
		map_info_ = info;

		occupancy_map_ready_ = true;
	}

	virtual bool isValid(const ompl::base::State *state) const
	{
		if(occupancy_map_ready_)
		{
			Position2Dd pos;

			pos.x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];;
			pos.y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

			Position2D pos_on_map;
			Position2D pos_on_paddedmap;
			pos_on_map = MapUtils::CoordinatesFromMapWorldToMap(pos, map_info_);
			pos_on_paddedmap = MapUtils::CoordinatesFromOriginalToPadded(pos_on_map, map_info_);

			//std::cout << "check point: " << pos_on_map.x << " , " << pos_on_map.y << std::endl;
			if(map_info_.vector_map)
				return ImageUtils::IsPointNonObstacle(occupancy_map_, cv::Point(pos_on_map.x, pos_on_map.y));
			else
				return ImageUtils::IsPointNonObstacle(occupancy_map_, cv::Point(pos_on_paddedmap.x, pos_on_paddedmap.y));
		}
		else
			return true;
	}
};

}

#endif /* PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_ */
