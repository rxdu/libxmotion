/*
 * qtree_vis.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: rdu
 */

#include "vis/qtree_vis.h"

using namespace srcl_ctrl;
using namespace cv;

/*
 * @param tree : reference to the tree to be visualized
 * @param _src : padded image from the tree builder
 * @param _dst : result image to be shown
 * @param vis_type : visualization type - FREE_SPACE, OCCU_SPACE or ALL_SPACE
 */
void Vis::VisQuadTree(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst, TreeVisType vis_type)
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
			Point top_left(tree.root_node_->bbox_.x.min, tree.root_node_->bbox_.y.min);
			Point top_right(tree.root_node_->bbox_.x.max,tree.root_node_->bbox_.y.min);
			Point bot_left(tree.root_node_->bbox_.x.min,tree.root_node_->bbox_.y.max);
			Point bot_right(tree.root_node_->bbox_.x.max,tree.root_node_->bbox_.y.max);

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
						Point top_left(parent->child_nodes_[i]->bbox_.x.min, parent->child_nodes_[i]->bbox_.y.min);
						Point top_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.min);
						Point bot_left(parent->child_nodes_[i]->bbox_.x.min,parent->child_nodes_[i]->bbox_.y.max);
						Point bot_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.max);

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
							Point top_left(parent->child_nodes_[i]->bbox_.x.min, parent->child_nodes_[i]->bbox_.y.min);
							Point top_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.min);
							Point bot_left(parent->child_nodes_[i]->bbox_.x.min,parent->child_nodes_[i]->bbox_.y.max);
							Point bot_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.max);

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

void Vis::VisQTreeWithDummies(const QuadTree& tree, cv::InputArray _src, cv::OutputArray _dst)
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
			Point top_left(tree.root_node_->bbox_.x.min, tree.root_node_->bbox_.y.min);
			Point top_right(tree.root_node_->bbox_.x.max,tree.root_node_->bbox_.y.min);
			Point bot_left(tree.root_node_->bbox_.x.min,tree.root_node_->bbox_.y.max);
			Point bot_right(tree.root_node_->bbox_.x.max,tree.root_node_->bbox_.y.max);

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
					Point top_left(parent->child_nodes_[i]->bbox_.x.min, parent->child_nodes_[i]->bbox_.y.min);
					Point top_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.min);
					Point bot_left(parent->child_nodes_[i]->bbox_.x.min,parent->child_nodes_[i]->bbox_.y.max);
					Point bot_right(parent->child_nodes_[i]->bbox_.x.max,parent->child_nodes_[i]->bbox_.y.max);

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
		Point top_left((*it)->bbox_.x.min, (*it)->bbox_.y.min);
		Point top_right((*it)->bbox_.x.max,(*it)->bbox_.y.min);
		Point bot_left((*it)->bbox_.x.min,(*it)->bbox_.y.max);
		Point bot_right((*it)->bbox_.x.max,(*it)->bbox_.y.max);

		line(src_img_color, top_left, top_right, Scalar(255,144,30));
		line(src_img_color, top_right, bot_right, Scalar(255,144,30));
		line(src_img_color, bot_right, bot_left, Scalar(255,144,30));
		line(src_img_color, bot_left, top_left, Scalar(255,144,30));
	}

	src_img_color.copyTo(dst);
}

void Vis::VisQTreeSingleNode(const QuadTreeNode& node, cv::InputArray _src, cv::OutputArray _dst)
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

void Vis::VisQTreeNodes(const std::vector<QuadTreeNode*>& nodes, cv::InputArray _src, cv::OutputArray _dst)
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


