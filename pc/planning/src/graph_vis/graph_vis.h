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

#include "quadtree/quad_tree.h"
#include "square_grid/square_grid.h"
#include "graph/graph.h"

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
	// quad-tree visualization
	void VisQuadTree(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type);
	void VisQTreeWithDummies(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst);
	void VisQTreeSingleNode(const QuadTreeNode& node, cv::InputArray _src, cv::OutputArray _dst);
	void VisQTreeNodes(const std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst);

	// square grid visualization
	void VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	void VisAbstractSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	void VisSquareGrid(const SquareGrid& grid, cv::InputArray _src, cv::OutputArray _dst);

	// graph visualization
	void VisQTreeGraph(const Graph_t<QuadTreeNode*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id, bool show_cost);
	void VisQTreeGraphPath(const std::vector<Vertex_t<QuadTreeNode*>*>& vertices, cv::InputArray _src, cv::OutputArray _dst);

	void VisSquareGridGraph(const Graph_t<SquareCell*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id);
	void VisSquareGridPath(const std::vector<Vertex_t<SquareCell*>*>& path, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
