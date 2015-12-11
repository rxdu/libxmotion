#include <cmath>
#include "quad_tree.h"

//#ifdef DEBUG
#include <iostream>
//#endif

using namespace srcl_ctrl;

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

/*********************************************************/
/*              Implementation of Tree Node              */
/*********************************************************/

TreeNode::TreeNode(BoundingBox bound, OccupancyType occupancy):
		occupancy_(occupancy),dummy_root_(nullptr)
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


