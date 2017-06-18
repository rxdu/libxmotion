/*
 * rrt_vis.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRT_VIS_RRT_VIS_H_
#define PLANNING_SRC_RRT_VIS_RRT_VIS_H_

#include "opencv2/opencv.hpp"

#include "common/planning_types.h"
#include "map/map_info.h"
#include "graph/graph.h"
#include "rrtstar/rrt_node.h"

namespace srcl_ctrl {

namespace RRTVis {
	void VisRRTPath(const std::vector<Position2Dd>& path, MapInfo info, cv::InputArray _src, cv::OutputArray _dst);
	void VisRRTGraph(const Graph_t<RRTNode>& graph, MapInfo info, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* PLANNING_SRC_RRT_VIS_RRT_VIS_H_ */
