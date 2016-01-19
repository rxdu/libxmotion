/*
 * graph_vis.h
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

#ifndef SRC_VISUALIZER_GRAPH_VIS_H_
#define SRC_VISUALIZER_GRAPH_VIS_H_

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quad_tree.h"
#include "graph.h"

namespace srcl_ctrl {

enum class TreeVisType
{
	FREE_SPACE,
	OCCU_SPACE,
	ALL_SPACE
};

class GraphVis
{
public:
	GraphVis();
	~GraphVis();

private:
	void DrawEdge(cv::Point pt1, cv::Point pt2, cv::Mat img);
	void DrawQTreeNode(const QuadTreeNode *node, cv::Mat img);

public:
	void DrawQuadTree(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type);
	void DrawQTreeWithDummies(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst);
	void DrawQTreeSingleNode(QuadTreeNode* node, cv::InputArray _src, cv::OutputArray _dst);
	void DrawQTreeNodes(std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst);

	void DrawQTreeGraph(Graph<QuadTreeNode> *graph, QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
