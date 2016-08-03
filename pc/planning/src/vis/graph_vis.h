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
private:
	static cv::Scalar bk_color_;		// background color
	static cv::Scalar ln_color_;		// line color
	static cv::Scalar obs_color_;		// obstacle color
	static cv::Scalar aoi_color_;		// area of interest color
	static cv::Scalar start_color_; 	// starting cell color
	static cv::Scalar finish_color_;	// finishing cell color

public:
	// quad-tree visualization
	static void VisQuadTree(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type);
	static void VisQTreeWithDummies(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst);
	static void VisQTreeSingleNode(const QuadTreeNode& node, cv::InputArray _src, cv::OutputArray _dst);
	static void VisQTreeNodes(const std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst);

	// square grid visualization
	static void VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	static void VisAbstractSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	static void VisSquareGrid(const SquareGrid& grid, cv::InputArray _src, cv::OutputArray _dst);

	// graph visualization
	static void VisQTreeGraph(const Graph_t<QuadTreeNode*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id, bool show_cost);
	static void VisQTreeGraphPath(const std::vector<Vertex_t<QuadTreeNode*>*>& vertices, cv::InputArray _src, cv::OutputArray _dst);

	static void VisSquareGridGraph(const Graph_t<SquareCell*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id);
	static void VisSquareGridPath(const std::vector<Vertex_t<SquareCell*>*>& path, cv::InputArray _src, cv::OutputArray _dst);
};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
