/*
 * graph_vis.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

#include <iostream>

#include "graph_vis.h"

using namespace srcl_ctrl;
using namespace cv;

GraphVis::GraphVis()
{

}

GraphVis::~GraphVis()
{

}

/*
 * @param *tree : pointer to the tree to be visualized
 * @param _src : padded image from the tree builder
 * @param _dst : result image to be shown
 * @param vis_type : visualization type - FREE_SPACE, OCCU_SPACE or ALL_SPACE
 */
void GraphVis::DrawQuadTree(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	if(tree != nullptr)
	{
		std::vector<TreeNode*> parent_nodes;

		for(int i = 0; i < tree->tree_depth_; i++)
		{
			if(i == 0)
			{
				Point top_left(tree->root_node_->bounding_box_.x.min, tree->root_node_->bounding_box_.y.min);
				Point top_right(tree->root_node_->bounding_box_.x.max,tree->root_node_->bounding_box_.y.min);
				Point bot_left(tree->root_node_->bounding_box_.x.min,tree->root_node_->bounding_box_.y.max);
				Point bot_right(tree->root_node_->bounding_box_.x.max,tree->root_node_->bounding_box_.y.max);

				line(src_img_color, top_left, top_right, Scalar(0,255,0));
				line(src_img_color, top_right, bot_right, Scalar(0,255,0));
				line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
				line(src_img_color, bot_left, top_left, Scalar(0,255,0));

				if(tree->root_node_->node_type_ != NodeType::LEAF)
				{
					for(int i = 0; i < 4; i++)
					{
						parent_nodes.clear();
						parent_nodes.push_back(tree->root_node_);
					}
				}
				else
					break;
			}
			else
			{
				std::vector<TreeNode*> inner_nodes;

				while(!parent_nodes.empty())
				{
					TreeNode* parent = parent_nodes.at(0);

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

#ifdef DEBUG
						if(parent->child_nodes_[i]->node_type_ == NodeType::LEAF && parent->child_nodes_[i]->occupancy_ == OccupancyType::FREE)
						{
							int thickness = -1;
							int lineType = 8;
							unsigned int x,y;
							x = parent->child_nodes_[i]->bounding_box_.x.min +
									(parent->child_nodes_[i]->bounding_box_.x.max - parent->child_nodes_[i]->bounding_box_.x.min + 1)/2;
							y = parent->child_nodes_[i]->bounding_box_.y.min +
									(parent->child_nodes_[i]->bounding_box_.y.max - parent->child_nodes_[i]->bounding_box_.y.min + 1)/2;
							Point center(x,y);
							circle( src_img_color,
									center,
									5,
									Scalar( 0, 0, 255 ),
									thickness,
									lineType);
						}
#endif
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
	}

	src_img_color.copyTo(dst);
}

void GraphVis::DrawQTreeWithDummies(QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	if(tree != nullptr)
	{
		// first draw dummies
		std::vector<TreeNode*> parent_nodes;

		for(int i = 0; i < tree->tree_depth_; i++)
		{
			if(i == 0)
			{
				Point top_left(tree->root_node_->bounding_box_.x.min, tree->root_node_->bounding_box_.y.min);
				Point top_right(tree->root_node_->bounding_box_.x.max,tree->root_node_->bounding_box_.y.min);
				Point bot_left(tree->root_node_->bounding_box_.x.min,tree->root_node_->bounding_box_.y.max);
				Point bot_right(tree->root_node_->bounding_box_.x.max,tree->root_node_->bounding_box_.y.max);

				line(src_img_color, top_left, top_right, Scalar(0,255,0));
				line(src_img_color, top_right, bot_right, Scalar(0,255,0));
				line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
				line(src_img_color, bot_left, top_left, Scalar(0,255,0));

				if(tree->root_node_->node_type_ != NodeType::LEAF)
				{
					for(int i = 0; i < 4; i++)
					{
						parent_nodes.clear();
						parent_nodes.push_back(tree->root_node_);
					}
				}
				else
					break;
			}
			else
			{
				std::vector<TreeNode*> inner_nodes;

				while(!parent_nodes.empty())
				{
					TreeNode* parent = parent_nodes.at(0);

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
		std::vector<TreeNode*>::iterator it;
		for(it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
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
	}

	src_img_color.copyTo(dst);
}

void GraphVis::DrawQTreeNode(TreeNode *node, cv::Mat img)
{
	if(node!=nullptr)
	{
		int thickness = -1;
		int lineType = 8;
		unsigned int x,y;
		x = node->bounding_box_.x.min +
				(node->bounding_box_.x.max - node->bounding_box_.x.min + 1)/2;
		y = node->bounding_box_.y.min +
				(node->bounding_box_.y.max - node->bounding_box_.y.min + 1)/2;
		Point center(x,y);
		circle( img,
				center,
				3,
				Scalar( 0, 0, 255 ),
				thickness,
				lineType);
	}
}

void GraphVis::DrawEdge(cv::Point pt1, cv::Point pt2, cv::Mat img)
{
	int thickness = 1;
	int lineType = 8;

	line( img,
			pt1,
			pt2,
			Scalar( 237, 149, 100 ),
			thickness,
			lineType);
}

void GraphVis::DrawQTreeSingleNode(TreeNode* node, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	if(node!=nullptr)
	{
		int thickness = -1;
		int lineType = 8;
		unsigned int x,y;
		x = node->bounding_box_.x.min +
				(node->bounding_box_.x.max - node->bounding_box_.x.min + 1)/2;
		y = node->bounding_box_.y.min +
				(node->bounding_box_.y.max - node->bounding_box_.y.min + 1)/2;
		Point center(x,y);
		circle( dst,
				center,
				10,
				Scalar( 0, 0, 255 ),
				thickness,
				lineType);
	}
}

void GraphVis::DrawQTreeNodes(std::vector<TreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	uint64_t x, y;
	int thickness = -1;
	int lineType = 8;

	std::vector<TreeNode*>::iterator it;
	for(it = nodes.begin(); it != nodes.end(); it++)
	{
		x = (*it)->bounding_box_.x.min +
				((*it)->bounding_box_.x.max - (*it)->bounding_box_.x.min + 1)/2;
		y = (*it)->bounding_box_.y.min +
				((*it)->bounding_box_.y.max - (*it)->bounding_box_.y.min + 1)/2;
		Point center(x,y);
		circle( dst,
				center,
				5,
				Scalar( 0, 128, 255 ),
				thickness,
				lineType);
	}
}

//void GraphVis::DrawQTreeGraph(Graph *graph, QuadTree *tree, cv::InputArray _src, cv::OutputArray _dst)
//{
//	Mat src = _src.getMat();
//	_dst.create(_src.size(), _src.type());
//	Mat dst = _dst.getMat();
//	src.copyTo(dst);
//
//	std::vector<Vertex*> vertices;
//	std::vector<Vertex*>::iterator itv;
//
//	vertices = graph->GetGraphVertices();
//
//	for(itv = vertices.begin(); itv != vertices.end(); itv++)
//	{
//		DrawQTreeNode((*itv)->node_,dst);
//
//		// current vertex center coordinate
//		uint64_t x1,y1,x2,y2;
//		x1 = (*itv)->node_->bounding_box_.x.min +
//				((*itv)->node_->bounding_box_.x.max - (*itv)->node_->bounding_box_.x.min + 1)/2;
//		y1 = (*itv)->node_->bounding_box_.y.min +
//				((*itv)->node_->bounding_box_.y.max - (*itv)->node_->bounding_box_.y.min + 1)/2;
//
//		std::vector<Edge>::iterator ite;
//		for(ite = (*itv)->adj_.begin(); ite != (*itv)->adj_.end(); ite++)
//		{
//			// neighbor vertices center coordinate
//			TreeNode* n = (*ite).dst_->node_;
//
//			x2 = n->bounding_box_.x.min +
//					(n->bounding_box_.x.max - n->bounding_box_.x.min + 1)/2;
//			y2 = n->bounding_box_.y.min +
//					(n->bounding_box_.y.max - n->bounding_box_.y.min + 1)/2;
//
//			DrawEdge(Point(x1,y1), Point(x2,y2), dst);
//		}
//	}
//
////	std::cout<<"number of vertices "<< vertices.size()<<std::endl;
//}
