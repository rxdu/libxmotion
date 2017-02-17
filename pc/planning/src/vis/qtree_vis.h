/*
 * qtree_vis.h
 *
 *  Created on: Feb 9, 2017
 *      Author: rdu
 */

#ifndef PLANNING_SRC_VIS_QTREE_VIS_H_
#define PLANNING_SRC_VIS_QTREE_VIS_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "graph/graph.h"
#include "geometry/quadtree/quad_tree.h"

namespace srcl_ctrl {

enum class TreeVisType
{
	FREE_SPACE,
	OCCU_SPACE,
	ALL_SPACE
};

namespace Vis
{
	// quad-tree visualization
	void VisQuadTree(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type);
	void VisQTreeWithDummies(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst);
	void VisQTreeSingleNode(const QuadTreeNode& node, cv::InputArray _src, cv::OutputArray _dst);
	void VisQTreeNodes(const std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* PLANNING_SRC_VIS_QTREE_VIS_H_ */
