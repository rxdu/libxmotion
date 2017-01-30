/*
 * graph_vis.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

#include <iostream>
#include <string>

#include "vis/graph_vis.h"
#include "vis/vis_utils.h"

using namespace srcl_ctrl;
using namespace cv;

cv::Scalar GraphVis::bk_color_ = Scalar(255,255,255);
cv::Scalar GraphVis::ln_color_ = Scalar(Scalar(0,0,0));
cv::Scalar GraphVis::obs_color_ = Scalar(Scalar(0,102,204));
cv::Scalar GraphVis::aoi_color_ = Scalar(Scalar(0,255,255));
cv::Scalar GraphVis::start_color_ = Scalar(0,0,255);
cv::Scalar GraphVis::finish_color_ = Scalar(153,76,0);

/*
 * @param tree : reference to the tree to be visualized
 * @param _src : padded image from the tree builder
 * @param _dst : result image to be shown
 * @param vis_type : visualization type - FREE_SPACE, OCCU_SPACE or ALL_SPACE
 */
void GraphVis::VisQuadTree(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	std::vector<QuadTreeNode*> parent_nodes;

	for(int i = 0; i < tree.tree_depth_; i++)
	{
		if(i == 0)
		{
			Point top_left(tree.root_node_->bounding_box_.x.min, tree.root_node_->bounding_box_.y.min);
			Point top_right(tree.root_node_->bounding_box_.x.max,tree.root_node_->bounding_box_.y.min);
			Point bot_left(tree.root_node_->bounding_box_.x.min,tree.root_node_->bounding_box_.y.max);
			Point bot_right(tree.root_node_->bounding_box_.x.max,tree.root_node_->bounding_box_.y.max);

			line(src_img_color, top_left, top_right, Scalar(0,255,0));
			line(src_img_color, top_right, bot_right, Scalar(0,255,0));
			line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
			line(src_img_color, bot_left, top_left, Scalar(0,255,0));

			if(tree.root_node_->node_type_ != NodeType::LEAF)
			{
				for(int i = 0; i < 4; i++)
				{
					parent_nodes.clear();
					parent_nodes.push_back(tree.root_node_);
				}
			}
			else
				break;
		}
		else
		{
			std::vector<QuadTreeNode*> inner_nodes;

			while(!parent_nodes.empty())
			{
				QuadTreeNode* parent = parent_nodes.at(0);

				for(int i = 0; i < 4; i++)
				{
					if(vis_type == TreeVisType::ALL_SPACE)
					{
						Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
						Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
						Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
						Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

						line(src_img_color, top_left, top_right, Scalar(0,255,0));
						line(src_img_color, top_right, bot_right, Scalar(0,255,0));
						line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
						line(src_img_color, bot_left, top_left, Scalar(0,255,0));
					}
					else
					{
						OccupancyType disp_type;

						if(vis_type == TreeVisType::FREE_SPACE)
							disp_type = OccupancyType::FREE;
						else
							disp_type = OccupancyType::OCCUPIED;

						if(parent->child_nodes_[i]->occupancy_ == disp_type){
							Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
							Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
							Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
							Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

							line(src_img_color, top_left, top_right, Scalar(0,255,0));
							line(src_img_color, top_right, bot_right, Scalar(0,255,0));
							line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
							line(src_img_color, bot_left, top_left, Scalar(0,255,0));
						}
					}

					if(parent->child_nodes_[i]->node_type_ == NodeType::INNER)
						inner_nodes.push_back(parent->child_nodes_[i]);
				}

				// delete the processed node
				parent_nodes.erase(parent_nodes.begin());
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes = inner_nodes;
		}
	}

	src_img_color.copyTo(dst);
}

void GraphVis::VisQTreeWithDummies(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	// first draw dummies
	std::vector<QuadTreeNode*> parent_nodes;

	for(int i = 0; i < tree.tree_depth_; i++)
	{
		if(i == 0)
		{
			Point top_left(tree.root_node_->bounding_box_.x.min, tree.root_node_->bounding_box_.y.min);
			Point top_right(tree.root_node_->bounding_box_.x.max,tree.root_node_->bounding_box_.y.min);
			Point bot_left(tree.root_node_->bounding_box_.x.min,tree.root_node_->bounding_box_.y.max);
			Point bot_right(tree.root_node_->bounding_box_.x.max,tree.root_node_->bounding_box_.y.max);

			line(src_img_color, top_left, top_right, Scalar(0,255,0));
			line(src_img_color, top_right, bot_right, Scalar(0,255,0));
			line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
			line(src_img_color, bot_left, top_left, Scalar(0,255,0));

			if(tree.root_node_->node_type_ != NodeType::LEAF)
			{
				for(int i = 0; i < 4; i++)
				{
					parent_nodes.clear();
					parent_nodes.push_back(tree.root_node_);
				}
			}
			else
				break;
		}
		else
		{
			std::vector<QuadTreeNode*> inner_nodes;

			while(!parent_nodes.empty())
			{
				QuadTreeNode* parent = parent_nodes.at(0);

				for(int i = 0; i < 4; i++)
				{
					Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
					Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
					Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
					Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

					if(parent->child_nodes_[i]->node_type_ != NodeType::LEAF){
						line(src_img_color, top_left, top_right, Scalar(1,97,230));
						line(src_img_color, top_right, bot_right, Scalar(1,97,230));
						line(src_img_color, bot_right, bot_left, Scalar(1,97,230));
						line(src_img_color, bot_left, top_left, Scalar(1,97,230));
					}
					//						if(parent->child_nodes_[i]->node_type_ == NodeType::INNER)
					inner_nodes.push_back(parent->child_nodes_[i]);
				}

				// delete the processed node
				parent_nodes.erase(parent_nodes.begin());
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes = inner_nodes;
		}
	}

	// then draw real leaves
	for(auto it = tree.leaf_nodes_.begin(); it != tree.leaf_nodes_.end(); it++)
	{
		Point top_left((*it)->bounding_box_.x.min, (*it)->bounding_box_.y.min);
		Point top_right((*it)->bounding_box_.x.max,(*it)->bounding_box_.y.min);
		Point bot_left((*it)->bounding_box_.x.min,(*it)->bounding_box_.y.max);
		Point bot_right((*it)->bounding_box_.x.max,(*it)->bounding_box_.y.max);

		line(src_img_color, top_left, top_right, Scalar(255,144,30));
		line(src_img_color, top_right, bot_right, Scalar(255,144,30));
		line(src_img_color, bot_right, bot_left, Scalar(255,144,30));
		line(src_img_color, bot_left, top_left, Scalar(255,144,30));
	}

	src_img_color.copyTo(dst);
}

void GraphVis::VisQTreeSingleNode(const QuadTreeNode& node, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	int thickness = -1;
	int lineType = 8;

	Point center(node.location_.x,node.location_.y);
	circle( dst,
			center,
			10,
			Scalar( 0, 0, 255 ),
			thickness,
			lineType);
}

void GraphVis::VisQTreeNodes(const std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	uint64_t x, y;
	int thickness = -1;
	int lineType = 8;

	for(auto it = nodes.begin(); it != nodes.end(); it++)
	{
		Point center((*it)->location_.x,(*it)->location_.y);
		circle( dst,
				center,
				5,
				Scalar( 0, 128, 255 ),
				thickness,
				lineType);
	}
}

void GraphVis::VisQTreeGraph(const Graph_t<QuadTreeNode*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id, bool show_cost)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	// draw all vertices
	std::vector<Vertex<QuadTreeNode*>*> vertices;
	vertices = graph.GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
		//DrawNodeCenter(center, dst);
		VisUtils::DrawPoint(dst, center);

		// current vertex center coordinate
		uint64_t x1,y1,x2,y2;
		x1 = (*itv)->bundled_data_->location_.x;
		y1 = (*itv)->bundled_data_->location_.y;

		if(show_id)
		{
			std::string id = std::to_string((*itv)->bundled_data_->data_id_);
			putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);
		}
	}

	// draw all edges
	std::vector<Edge_t<QuadTreeNode*>> edges;
	edges = graph.GetGraphUndirectedEdges();
	for(auto it = edges.begin(); it != edges.end(); it++)
	{
		uint64_t x1,y1,x2,y2;
		x1 = (*it).src_->bundled_data_->location_.x;
		y1 = (*it).src_->bundled_data_->location_.y;
		x2 = (*it).dst_->bundled_data_->location_.x;
		y2 = (*it).dst_->bundled_data_->location_.y;

		//DrawEdge(Point(x1,y1), Point(x2,y2), dst);
		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));

		// draw cost
		if(show_cost)
		{
			std::string str = std::to_string(static_cast<int>((*it).cost_));
			int tx = (x1 + x2)/2;
			int ty = (y1 + y2)/2;
			putText(dst, str ,Point(tx,ty), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
		}
	}
}

void GraphVis::VisQTreeGraphPath(const std::vector<Vertex_t<QuadTreeNode*>*>& vertices, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	std::vector<QuadTreeNode*> path_nodes;
	for(auto itn = vertices.begin(); itn != vertices.end(); itn++)
	{
		path_nodes.push_back((*itn)->bundled_data_);
	}

	// draw vertices
	uint64_t x, y;
	Scalar vertex_color;
	int thickness = 3;
	int lineType = 8;

	for(auto it = path_nodes.begin(); it != path_nodes.end(); it++)
	{
		Point center((*it)->location_.x,(*it)->location_.y);

		if(it == path_nodes.begin())
			vertex_color = Scalar( 0, 0, 255 );
		else if(it == path_nodes.end() - 1)
			vertex_color = Scalar( 153, 0, 0 );
		else
			vertex_color = Scalar( 153, 153, 0 );

		circle( dst,
				center,
				5,
				vertex_color,
				thickness,
				lineType);
	}

	// draw edges
	uint64_t x1,y1,x2,y2;
	int pathline_thickness = 2;
	for(auto it = path_nodes.begin(); it != path_nodes.end()-1; it++)
	{
		// neighbor vertices center coordinate
		x1 = (*it)->location_.x;
		y1 = (*it)->location_.y;

		x2 = (*(it+1))->location_.x;
		y2 = (*(it+1))->location_.y;

		line( dst,
				Point(x1,y1),
				Point(x2,y2),
				//Scalar( 237, 149, 100 ),
				Scalar( 255, 153, 51 ),
				pathline_thickness,
				lineType);
	}
}

void GraphVis::VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst)
{
	_dst.create(Size(grid.col_size_*grid.cell_size_, grid.row_size_*grid.cell_size_), CV_8UC3);
	Mat dst = _dst.getMat();
	dst = bk_color_;

	// fill cell color
	for(auto itc = grid.cells_.begin(); itc != grid.cells_.end(); itc++)
	{
		if((*itc).second->occu_ == OccupancyType::OCCUPIED)
			//FillSquareCellColor((*itc).second->bbox_, obs_color_, dst);
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, obs_color_);
		else if((*itc).second->occu_ == OccupancyType::INTERESTED)
			//FillSquareCellColor((*itc).second->bbox_, aoi_color_, dst);
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, aoi_color_);

		auto cell = (*itc);
		uint64_t x,y;
		x = cell.second->bbox_.x.min + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/2;
		x = x + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/6;
		y = cell.second->bbox_.y.min + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)/2;
		y = y + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)*3/7;

		std::string id = std::to_string(cell.second->data_id_);

		putText(dst, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
	}

	// draw grid lines
	line(dst, Point(0,0),Point(0,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	for(int i = 1; i <= grid.col_size_; i++){
		line(dst, Point(i*grid.cell_size_-1,0),Point(i*grid.cell_size_-1,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	}

	line(dst, Point(0,0),Point(grid.col_size_*grid.cell_size_-1,0),ln_color_, 1);
	for(int i = 1; i <= grid.row_size_; i++){
		line(dst, Point(0,i*grid.cell_size_-1),Point(grid.col_size_*grid.cell_size_-1,i*grid.cell_size_-1),ln_color_, 1);
	}
}

void GraphVis::VisSquareGrid(const SquareGrid& grid, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	std::vector<SquareCell*> cells;
	for(auto it = grid.cells_.begin(); it != grid.cells_.end(); it++)
	{
		cells.push_back((*it).second);
	}

	for(auto itc = cells.begin(); itc != cells.end(); itc++)
	{
		Point top_left((*itc)->bbox_.x.min, (*itc)->bbox_.y.min);
		Point top_right((*itc)->bbox_.x.max,(*itc)->bbox_.y.min);
		Point bot_left((*itc)->bbox_.x.min,(*itc)->bbox_.y.max);
		Point bot_right((*itc)->bbox_.x.max,(*itc)->bbox_.y.max);

		line(src_img_color, top_left, top_right, Scalar(0,255,0));
		line(src_img_color, top_right, bot_right, Scalar(0,255,0));
		line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
		line(src_img_color, bot_left, top_left, Scalar(0,255,0));

		/*auto cell = (*itc);
		uint64_t x,y;
		x = cell->bbox_.x.min + (cell->bbox_.x.max - cell->bbox_.x.min)/2;
		x = x + (cell->bbox_.x.max - cell->bbox_.x.min)/6;
		y = cell->bbox_.y.min + (cell->bbox_.y.max - cell->bbox_.y.min)/2;
		y = y + (cell->bbox_.y.max - cell->bbox_.y.min)*3/7;
		std::string id = std::to_string(cell->data_id_);

		putText(src_img_color, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
		*/
	}

	src_img_color.copyTo(dst);
}

void GraphVis::VisSquareGridGraph(const Graph_t<SquareCell*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	// draw all vertices
	std::vector<Vertex<SquareCell*>*> vertices;
	vertices = graph.GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
		//DrawNodeCenter(center,dst);
		VisUtils::DrawPoint(dst, center);

		// current vertex center coordinate
		uint64_t x1,y1,x2,y2;
		x1 = (*itv)->bundled_data_->location_.x;
		y1 = (*itv)->bundled_data_->location_.y;

		if(show_id) {
			if((*itv)->bundled_data_->data_id_ % 2 == 0)
			{
				std::string id = std::to_string((*itv)->bundled_data_->data_id_);
				putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);
			}
		}
		//putText(dst, std::to_string((int)(*itv)->potential_) ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);
	}

	// draw all edges
	auto edges = graph.GetGraphUndirectedEdges();
	for(auto it = edges.begin(); it != edges.end(); it++)
	{
		uint64_t x1,y1,x2,y2;
		x1 = (*it).src_->bundled_data_->location_.x;
		y1 = (*it).src_->bundled_data_->location_.y;
		x2 = (*it).dst_->bundled_data_->location_.x;
		y2 = (*it).dst_->bundled_data_->location_.y;

		//DrawEdge(Point(x1,y1), Point(x2,y2), dst);
		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
	}
}

void GraphVis::VisSquareGridNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, Vertex_t<SquareCell*>* start_vtx, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
	_dst.create(Size(grid.col_size_*grid.cell_size_, grid.row_size_*grid.cell_size_), CV_8UC3);
	Mat dst = _dst.getMat();
	dst = bk_color_;

	// draw potential filed
//	int thickness = 2;
//	int lineType = 8;
//
//	for(int i = 1; i <= 30; ++i) {
//		circle( dst,
//				Point(nav_field.field_center_->bundled_data_->location_.x, nav_field.field_center_->bundled_data_->location_.y),
//				95*i,
//				Scalar(rand() % 255,rand() % 255,rand() % 255),
//				thickness,
//				lineType );
//	}

	// fill cell color
	for(auto itc = grid.cells_.begin(); itc != grid.cells_.end(); itc++)
	{
		if((*itc).second->occu_ == OccupancyType::OCCUPIED)
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, obs_color_);
		//		else if((*itc).second->occu_ == OccupancyType::INTERESTED)
		//			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, aoi_color_);

		if(show_id) {
			auto cell = (*itc);
			uint64_t x,y;
			x = cell.second->bbox_.x.min + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/2;
			x = x + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/6;
			y = cell.second->bbox_.y.min + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)/2;
			y = y + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)*3/7;

			std::string id = std::to_string(cell.second->data_id_);

			putText(dst, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
		}
	}

	// draw grid lines
	line(dst, Point(0,0),Point(0,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	for(int i = 1; i <= grid.col_size_; i++){
		line(dst, Point(i*grid.cell_size_-1,0),Point(i*grid.cell_size_-1,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	}

	line(dst, Point(0,0),Point(grid.col_size_*grid.cell_size_-1,0),ln_color_, 1);
	for(int i = 1; i <= grid.row_size_; i++){
		line(dst, Point(0,i*grid.cell_size_-1),Point(grid.col_size_*grid.cell_size_-1,i*grid.cell_size_-1),ln_color_, 1);
	}

	// draw all vertices
	std::vector<Vertex<SquareCell*>*> vertices;
	vertices = nav_field.field_graph_->GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
		VisUtils::DrawPoint(dst, center);

		// current vertex center coordinate
		uint64_t x1,y1,x2,y2;
		x1 = (*itv)->bundled_data_->location_.x;
		y1 = (*itv)->bundled_data_->location_.y;

//		if(show_id) {
//			if((*itv)->bundled_data_->data_id_ % 2 == 0)
//			{
//				std::string id = std::to_string((*itv)->bundled_data_->data_id_);
//				putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);
//			}
//		}

		// display potential value
		putText(dst, std::to_string((int)(*itv)->potential_) ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);

//		if((*itv)->potential_ > start_vtx->potential_)
//			VisUtils::FillRectangularArea(dst, (*itv)->bundled_data_->bbox_, Scalar(211,211,211));
	}

	// draw all edges
	auto edges = nav_field.field_graph_->GetGraphUndirectedEdges();
	for(auto it = edges.begin(); it != edges.end(); it++)
	{
		uint64_t x1,y1,x2,y2;
		x1 = (*it).src_->bundled_data_->location_.x;
		y1 = (*it).src_->bundled_data_->location_.y;
		x2 = (*it).dst_->bundled_data_->location_.x;
		y2 = (*it).dst_->bundled_data_->location_.y;

		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
	}
}

void GraphVis::VisSquareGridLocalNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, Vertex_t<SquareCell*>* center_vtx, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	auto nbs = const_cast<SquareGrid&>(grid).GetNeighboursWithinRange(center_vtx->bundled_data_->data_id_,2);
	for(auto& n : nbs) {
		auto vtx = nav_field.field_graph_->GetVertexFromID(n->data_id_);
		if(vtx->potential_ > center_vtx->potential_)
			VisUtils::FillRectangularArea(dst, vtx->bundled_data_->bbox_, Scalar(211,211,211));
	}
//	if((*itv)->potential_ > start_vtx->potential_)
//		VisUtils::FillRectangularArea(dst, (*itv)->bundled_data_->bbox_, Scalar(211,211,211));
}

void GraphVis::VisSquareGridPath(const std::vector<Vertex_t<SquareCell*>*>& path, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	// draw starting and finishing cell
	auto cell_s = path[0]->bundled_data_;
	uint64_t x,y;
	x = cell_s->location_.x;
	x = x - (cell_s->bbox_.x.max - cell_s->bbox_.x.min)/8;
	y = cell_s->location_.y;
	y = y + (cell_s->bbox_.y.max - cell_s->bbox_.y.min)/8;
	//FillSquareCellColor(cell_s->bbox_, start_color_, dst);
	VisUtils::FillRectangularArea(dst, cell_s->bbox_, start_color_);
	putText(dst, "S" ,Point(x,y), CV_FONT_NORMAL, 1, Scalar(0,0,0),1,1);

	auto cell_f = (*(path.end()-1))->bundled_data_;
	x = cell_f->location_.x;
	x = x - (cell_f->bbox_.x.max - cell_f->bbox_.x.min)/8;
	y = cell_f->location_.y;
	y = y + (cell_f->bbox_.y.max - cell_f->bbox_.y.min)/8;
	//FillSquareCellColor(cell_f->bbox_, finish_color_, dst);
	VisUtils::FillRectangularArea(dst, cell_f->bbox_, finish_color_);
	putText(dst, "F" ,Point(x,y), CV_FONT_NORMAL, 1, Scalar(0,0,0),1,1);

	// draw path
	uint64_t x1,y1,x2,y2;
	int thickness = 3;
	int lineType = 8;
	int pathline_thickness = 2;

	for(auto it = path.begin(); it != path.end()-1; it++)
	{
		// consecutive cells
		auto cell1 = (*it)->bundled_data_;
		auto cell2 = (*(it+1))->bundled_data_;

		// center coordinates
		x1 = cell1->location_.x;
		y1 = cell1->location_.y;

		x2 = cell2->location_.x;
		y2 = cell2->location_.y;

		line( dst,
				Point(x1,y1),
				Point(x2,y2),
				//Scalar( 237, 149, 100 ),
				Scalar( 255, 153, 51 ),
				pathline_thickness,
				lineType);
	}
}
