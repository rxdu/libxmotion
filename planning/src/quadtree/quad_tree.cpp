#include <cmath>
#include <algorithm>
#include "quad_tree.h"

//#ifdef DEBUG
#include <iostream>
//#endif

using namespace srcl_ctrl;

/*********************************************************/
/*              Implementation of Quad Tree              */
/*********************************************************/
QuadTree::QuadTree():MAX_DEPTH(32),
		tree_depth_(0),
		cell_res_(0),
		root_node_(nullptr),
		node_manager_(nullptr)
{
}

QuadTree::QuadTree(uint16_t padded_img_size, uint8_t depth):MAX_DEPTH(32),
		tree_depth_(0),
		root_node_(nullptr)
{
	if(depth  > MAX_DEPTH)
		depth = MAX_DEPTH;

	node_manager_  = new QTreeNodeManager(depth);

	cell_res_ = padded_img_size/(uint16_t(pow(2,depth)));

//	std::cout << "image size: "<< int(image_size )<< std::endl;
//	std::cout << "depth: "<< int(depth) << std::endl;
//	std::cout << "tree cell size: "<< int(cell_res_ )<< std::endl;
}

QuadTree::~QuadTree()
{
	// TODO
	// Think about how to free memory of the whole tree
//	if(root_node_ != nullptr)
//		delete(root_node_);

	if(node_manager_ != nullptr)
		delete(node_manager_);
}

TreeNode* QuadTree::GetNodeAtPosition(uint16_t pixel_x, uint16_t pixel_y)
{
	uint16_t x,y;

	if(node_manager_ != nullptr)
	{
		x = pixel_x / cell_res_;
		y = pixel_y / cell_res_;

//		std::cout << "(x, y): "<<x<<" , "<<y<<std::endl;
//		std::cout << "Node list size: "<<node_manager_->side_node_num_<<std::endl;
//		if(node_manager_->GetNodeReference(x , y) == nullptr)
//			std::cout << "get empty node"<<std::endl;

		return node_manager_->GetNodeReference(x , y);
	}
	else
		return nullptr;
}

std::vector<TreeNode*> QuadTree::GetDummyNeighbours(TreeNode* dummy_leaf)
{
	std::vector<TreeNode*> dummy_neighbours;
	uint16_t x,y;
	TreeNode* leaf = dummy_leaf;

	x = ((leaf->bounding_box_.x.min + leaf->bounding_box_.x.max + 1)/2)/cell_res_;
	y = ((leaf->bounding_box_.y.min + leaf->bounding_box_.y.max + 1)/2)/cell_res_;

	dummy_neighbours.push_back(node_manager_->GetNodeReference(x-1,y-1));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x,y-1));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y-1));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x-1,y));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x-1,y+1));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x,y+1));
	dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y+1));

	return dummy_neighbours;
}

std::vector<TreeNode*> QuadTree::FindNeighbours(TreeNode* node)
{
//	std::vector<TreeNode*> leaf_dummies;
	std::vector<TreeNode*> dummy_neighbours;
	std::vector<TreeNode*> neighbours;

//	leaf_dummies.clear();
//	neighbour_dummies.clear();
//	neighbours.clear();

	// if the node is not at the highest resolution, find
	//	all its dummy leaf nodes
	int dummy_depth = 0;
	if(node->has_dummy_)
	{
		TreeNode* node_index = node;

		// check how many levels of dummy nodes the node has
		while(node_index->node_type_!=NodeType::DUMMY_LEAF)
		{
			dummy_depth++;
			node_index = node_index->child_nodes_[0];
		}

//		std::cout << "dummy node depth: "<<dummy_depth<<std::endl;
//
//		// find all leaf dummy nodes
//		std::vector<TreeNode*> parent_nodes;
//		parent_nodes.push_back(node);
//
//		for(int i = 0; i < dummy_depth; i++)
//		{
//			std::vector<TreeNode*> inner_nodes;
//
//			while(!parent_nodes.empty())
//			{
//				TreeNode* parent = parent_nodes.at(0);
//
//				for(int i = 0; i < 4; i++)
//				{
//					if(parent->child_nodes_[i]->node_type_ == NodeType::DUMMY_LEAF)
//						leaf_dummies.push_back(parent->child_nodes_[i]);
//					else
//						inner_nodes.push_back(parent->child_nodes_[i]);
//				}
//
//				// delete the processed node
//				parent_nodes.erase(parent_nodes.begin());
//			}
//
//			// prepare for next iteration
//			parent_nodes.clear();
//			parent_nodes = inner_nodes;
		}

//		std::cout << "dummy node num: "<<leaf_dummies.size()<<std::endl;
//	}
//	else
//	{
////		std::cout<<"no dummy"<<std::endl;
//		leaf_dummies.push_back(node);
//	}

	// find neighbour dummies around the leaf dummies
	if(dummy_depth == 0)
	{
		dummy_neighbours = GetDummyNeighbours(node);
	}
	else
	{
		// first find the coordinates of top left corner neighbour leaf dummy
		uint16_t x,y;
		x = (node->bounding_box_.x.min - 1)/cell_res_;
		y = (node->bounding_box_.y.min - 1)/cell_res_;

		uint16_t neighbour_side_num;
		neighbour_side_num = pow(2,dummy_depth)+2;

		// add first row
		for(int i = 0; i < neighbour_side_num; i++)
		{
			uint16_t x_loc, y_loc;
			x_loc = x + i;
			y_loc = y;

			if(x_loc >= 0 && x_loc < node_manager_->side_node_num_
					&& y_loc >= 0 && y_loc < node_manager_->side_node_num_)
				dummy_neighbours.push_back(node_manager_->GetNodeReference(x_loc,y_loc));
		}

		// add last row
		for(int i = 0; i < neighbour_side_num; i++)
		{
			uint16_t x_loc, y_loc;
			x_loc = x + i;
			y_loc = y + neighbour_side_num - 1;

			if(x_loc >= 0 && x_loc < node_manager_->side_node_num_
					&& y_loc >= 0 && y_loc < node_manager_->side_node_num_)
				dummy_neighbours.push_back(node_manager_->GetNodeReference(x_loc,y_loc));
		}

		// add left column
		for(int i = 0; i < neighbour_side_num - 2; i++)
		{
			uint16_t x_loc, y_loc;
			x_loc = x;
			y_loc = y + 1 + i;

			if(x_loc >= 0 && x_loc < node_manager_->side_node_num_
					&& y_loc >= 0 && y_loc < node_manager_->side_node_num_)
				dummy_neighbours.push_back(node_manager_->GetNodeReference(x_loc, y_loc));
		}

		// add right column
		for(int i = 0; i < neighbour_side_num - 2; i++)
		{
			uint16_t x_loc, y_loc;
			x_loc = x + neighbour_side_num - 1;
			y_loc = y + 1 + i;

			if(x_loc >= 0 && x_loc < node_manager_->side_node_num_
					&& y_loc >= 0 && y_loc < node_manager_->side_node_num_)
				dummy_neighbours.push_back(node_manager_->GetNodeReference(x_loc,y_loc));
		}
	}

	std::cout << "number of dummy neighbours: "<< dummy_neighbours.size() <<std::endl;

	// now find all dummy roots as neighbours in the quadtree
	neighbours.clear();
	std::vector<TreeNode*>::iterator it;
	for(it = dummy_neighbours.begin(); it != dummy_neighbours.end(); ++it)
	{
		TreeNode* qt_neighbour = (*it)->dummy_root_;

		bool existed = false;
		for(std::vector<TreeNode*>::iterator it_qt = neighbours.begin(); it_qt != neighbours.end(); it_qt++)
		{
			if(*(it_qt) == qt_neighbour)
			{
				existed = true;
				break;
			}
		}

		if(!existed)
			neighbours.push_back(qt_neighbour);
	}

	return neighbours;
//	return dummy_neighbours;
}

/*********************************************************/
/*          Data Structure for Neighbor Finding          */
/*********************************************************/

QTreeNodeManager::QTreeNodeManager(uint8_t tree_depth)
{
	tree_nodes_.clear();
	side_node_num_ = uint32_t(pow(2,tree_depth));
	tree_nodes_.resize(side_node_num_*side_node_num_);

//	std::cout << "node number: "<< tree_nodes_.size() << std::endl;
}

QTreeNodeManager::~QTreeNodeManager()
{

}

void QTreeNodeManager::SetNodeReference(uint16_t index_x, uint16_t index_y, TreeNode* node)
{
	tree_nodes_.at(index_y*side_node_num_ + index_x) = node;
}

TreeNode* QTreeNodeManager::GetNodeReference(uint16_t index_x, uint16_t index_y)
{
	return tree_nodes_.at(index_y*side_node_num_ + index_x);
}

/*********************************************************/
/*              Implementation of Tree Node              */
/*********************************************************/

TreeNode::TreeNode(BoundingBox bound, OccupancyType occupancy):
		occupancy_(occupancy),dummy_root_(this),has_dummy_(false)
{
	node_type_ = NodeType::INNER;

	bounding_box_.x.min = bound.x.min;
	bounding_box_.x.max = bound.x.max;
	bounding_box_.y.min = bound.y.min;
	bounding_box_.y.max = bound.y.max;

	child_nodes_[0] = nullptr;
	child_nodes_[1] = nullptr;
	child_nodes_[2] = nullptr;
	child_nodes_[3] = nullptr;
}

TreeNode::~TreeNode()
{

}

bool TreeNode::operator ==(const TreeNode* other)
{
	if(this != other)
		return false;
	else
		return true;
}

