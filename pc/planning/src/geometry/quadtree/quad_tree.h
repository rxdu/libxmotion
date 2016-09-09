#ifndef QUAD_TREE_
#define QUAD_TREE_

#include <common/planning_types.h>
#include <cstdint>
#include <vector>
#include <cstdint>

//#include "qtree_types.h"
#include "graph/bds_base.h"

namespace srcl_ctrl {

// Definition of Supporting Types
enum class NodeType
{
	INNER,
	LEAF,
	DUMMY_INNER,
	DUMMY_LEAF
};


// Definition of Tree Node
/*     Order of child nodes    */
/* 2 - top_left, 3 - top_right */
/* 0 - bot_left, 1 - bot_right */
class QuadTreeNode: public BDSBase<QuadTreeNode>
{
public:
	QuadTreeNode(BoundingBox bound, OccupancyType occupancy);
	~QuadTreeNode();

public:
	// Node contents
//	uint64_t node_id_;
	NodeType node_type_;
	OccupancyType occupancy_;
	Position2D location_;
	BoundingBox bounding_box_;

	// Pointers to child nodes
	QuadTreeNode* child_nodes_[4];

	// Extra information for neighbor search
	QuadTreeNode* dummy_root_;
	bool has_dummy_;

public:
	bool operator ==(const QuadTreeNode* other);
	double GetHeuristic(const QuadTreeNode& other_struct) const;
};

class QTreeNodeManager{
public:
	QTreeNodeManager(uint8_t tree_depth);
	~QTreeNodeManager();

private:
	std::vector<QuadTreeNode*> tree_nodes_;

public:
	uint16_t side_node_num_;

public:
	void SetNodeReference(uint16_t index_x, uint16_t index_y, QuadTreeNode* node);
	QuadTreeNode* GetNodeReference(uint16_t index_x, uint16_t index_y);
};

// Definition of Quad-Tree
class QuadTree{
public:
	QuadTree();
	QuadTree(uint16_t area_size, uint8_t depth);
	~QuadTree();

	typedef QuadTreeNode node_type;

public:
	// Basic tree information
	QuadTreeNode* root_node_;
	uint8_t tree_depth_;
	uint16_t cell_res_;
	const uint8_t MAX_DEPTH;

	// Extra information for neighbor search
	QTreeNodeManager* node_manager_;

	// Other data
	std::vector<QuadTreeNode*> leaf_nodes_;

private:
	std::vector<QuadTreeNode*> GetDummyNeighbours(QuadTreeNode* dummy_leaf);
	QuadTreeNode* GetNodeAtPosition(uint32_t pixel_x, uint32_t pixel_y);

public:
	uint64_t GetIDFromPosition(uint32_t x, uint32_t y);
	std::vector<QuadTreeNode*> FindNeighbours(QuadTreeNode* node);
};

}

#endif
