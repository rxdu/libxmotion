/*
 * rrt_vis.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRT_VIS_RRT_VIS_H_
#define PLANNING_SRC_RRT_VIS_RRT_VIS_H_

#include <vector>
#include <cstdint>

#include "opencv2/opencv.hpp"

#include "common/planning_types.h"
#include "map/map_info.h"

namespace srcl_ctrl {

class RRTVis {
private:
	static cv::Scalar bk_color_;		// background color
	static cv::Scalar ln_color_;		// line color
	static cv::Scalar obs_color_;		// obstacle color
	static cv::Scalar aoi_color_;		// area of interest color
	static cv::Scalar start_color_; 	// starting cell color
	static cv::Scalar finish_color_;	// finishing cell color

public:
	static void VisRRTPath(const std::vector<Position2Dd>& path, MapInfo info, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* PLANNING_SRC_RRT_VIS_RRT_VIS_H_ */
