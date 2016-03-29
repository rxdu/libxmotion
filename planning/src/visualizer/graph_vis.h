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
#include "square_grid.h"
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
	cv::Scalar bk_color_;		// background color
	cv::Scalar ln_color_;		// line color
	cv::Scalar obs_color_;		// obstacle color
	cv::Scalar aoi_color_;		// area of interest color
	cv::Scalar start_color_; 	// starting cell color
	cv::Scalar finish_color_;	// finishing cell color

private:
	void DrawEdge(cv::Point pt1, cv::Point pt2, cv::Mat img);
	void DrawNodeCenter(cv::Point pos, cv::Mat img);

	void FillSquareCellColor(BoundingBox bbox, cv::Scalar color, cv::Mat img);

public:
	// quadtree visualization
	void DrawQuadTree(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type);
	void DrawQTreeWithDummies(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst);
	void DrawQTreeSingleNode(QuadTreeNode* node, cv::InputArray _src, cv::OutputArray _dst);
	void DrawQTreeNodes(std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst);

	// square grid visualization
//	void DrawSquareGrid(SquareGrid* grid, cv::OutputArray _dst);
	void VisSquareGrid(SquareGrid* grid, cv::OutputArray _dst);
	void VisSquareGrid(SquareGrid* grid, cv::InputArray _src, cv::OutputArray _dst);

	// graph visualization
	void VisQTreeGraph(Graph<QuadTreeNode>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id, bool show_cost);
	void VisQTreeGraphPath(std::vector<Vertex<QuadTreeNode>*>& vertices, cv::InputArray _src, cv::OutputArray _dst);

	void VisSquareGridGraph(Graph<SquareCell>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id);
	void VisSquareGridPath(std::vector<Vertex<SquareCell>*>& path, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
