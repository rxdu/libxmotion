#ifndef QUAD_TREE_
#define QUAD_TREE_

#include <cstdint>
#include "qtree_types.h"

namespace srcl_ctrl {

// Definition of Tree Node
class TreeNode{
public:
	TreeNode(BoundingBox bound, OccupancyType occupancy);
	~TreeNode();

public:
	// Node contents
	NodeType type_;
	OccupancyType occupancy_;
	BoundingBox bounding_box_;

	// Pointers to child nodes
	/* 0 - top_left, 1 - top_right */
	/* 2 - bot_left, 3 - bot_right */
	TreeNode* child_nodes_[4];
};

// Definition of Quad-Tree
class QuadTree{
public:
	QuadTree();
	~QuadTree();

public:
	TreeNode* root_node_;
	unsigned int tree_depth_;

public:
};

}

#endif
