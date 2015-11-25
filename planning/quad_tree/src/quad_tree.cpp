#include "quad_tree.h"

using namespace srcl_ctrl;

/*********************************************************/
/*              Implementation of Quad Tree              */
/*********************************************************/
QuadTree::QuadTree(BoundingBox root_bound, bool root_occupied):
		tree_depth_(0),
		root_node_(new TreeNode(root_bound,root_occupied))
{

}

QuadTree::~QuadTree()
{
	if(root_node_ != nullptr)
		delete(root_node_);
}

/*********************************************************/
/*              Implementation of Tree Node              */
/*********************************************************/

TreeNode::TreeNode(BoundingBox bound, bool is_occupied):occupied_(is_occupied)
{
	type_ = NodeType::Node;

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


