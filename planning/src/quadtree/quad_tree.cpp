#include "quad_tree.h"

using namespace srcl_ctrl;

/*********************************************************/
/*              Implementation of Quad Tree              */
/*********************************************************/
QuadTree::QuadTree():tree_depth_(0), root_node_(nullptr)
{

}

QuadTree::~QuadTree()
{
	// TODO
	// Think about how to free memory of the whole tree
//	if(root_node_ != nullptr)
//		delete(root_node_);
}

/*********************************************************/
/*              Implementation of Tree Node              */
/*********************************************************/

TreeNode::TreeNode(BoundingBox bound, OccupancyType occupancy):
		occupancy_(occupancy)
{
	type_ = NodeType::INNER;

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


