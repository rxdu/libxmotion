#ifndef QUAD_TREE_
#define QUAD_TREE_

namespace srcl_ctrl {

// Definition of Supporting Types
enum class NodeType
{
	Node,
	Leaf
};

typedef struct
{
	unsigned long min;
	unsigned long max;
}NodeRange;

typedef struct
{
	NodeRange x;
	NodeRange y;
}BoundingBox;

// Definition of Tree Node
class TreeNode{
public:
	TreeNode(BoundingBox bound, bool is_occupied);
	~TreeNode();

public:
	NodeType type_;
	bool occupied_;
	/* 0 - top_left, 1 - top_right */
	/* 2 - bot_left, 3 - bot_right */
	TreeNode* child_nodes_[4];
	BoundingBox bounding_box_;
};

// Definition of Quad-Tree
class QuadTree{
public:
	QuadTree(BoundingBox root_bound, bool root_occupied);
	~QuadTree();

public:
	TreeNode* root_node_;
	unsigned int tree_depth_;

public:
};

}

#endif
