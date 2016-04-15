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

QuadTree::QuadTree(uint16_t image_size, uint8_t depth):MAX_DEPTH(32),
		tree_depth_(0),
		root_node_(nullptr)
{
	if(depth  > MAX_DEPTH)
		depth = MAX_DEPTH;

	node_manager_  = new QTreeNodeManager(depth);

	cell_res_ = image_size/(uint16_t(pow(2,depth)));
}

QuadTree::~QuadTree()
{
	// collect memory for all tree nodes
	std::vector<std::vector<QuadTreeNode*>> node_parents;

	// collect root level node
	std::vector<QuadTreeNode*> root_level;
	root_level.push_back(root_node_);
	node_parents.push_back(root_level);

	// collect lower level nodes
	for(int i = 0; i < this->tree_depth_ - 1; i++)
	{
		std::vector<QuadTreeNode*> nodes;

		for(auto it = node_parents[i].begin(); it != node_parents[i].end(); it++) {
			for(int i = 0; i < 4; i++)
				nodes.push_back((*it)->child_nodes_[i]);
		}

		node_parents.push_back(nodes);
	}
//	std::cout<<"total depth collected: " << node_parents.size() << std::endl;

	for(auto itr = node_parents.rbegin(); itr != node_parents.rend(); itr++)
	{
		for(auto ite = (*itr).begin(); ite != (*itr).end(); ite++)
		{
			for(int i = 0; i < 4; i++)
				delete (*ite)->child_nodes_[i];
		}
	}
	delete root_node_;

	delete node_manager_;
}

QuadTreeNode* QuadTree::GetNodeAtPosition(uint16_t pixel_x, uint16_t pixel_y)
{
	uint16_t x,y;

	if(node_manager_ != nullptr)
	{
		x = pixel_x / cell_res_;
		y = pixel_y / cell_res_;

		return node_manager_->GetNodeReference(x , y);
	}
	else
		return nullptr;
}

std::vector<QuadTreeNode*> QuadTree::GetDummyNeighbours(QuadTreeNode* dummy_leaf)
{
	std::vector<QuadTreeNode*> dummy_neighbours;
	uint16_t x,y;
	QuadTreeNode* leaf = dummy_leaf;

	x = ((leaf->bounding_box_.x.min + leaf->bounding_box_.x.max + 1)/2)/cell_res_;
	y = ((leaf->bounding_box_.y.min + leaf->bounding_box_.y.max + 1)/2)/cell_res_;

	QuadTreeNode* element;
	element = node_manager_->GetNodeReference(x-1,y-1);
	if(element != nullptr)
		dummy_neighbours.push_back(element);

	element = node_manager_->GetNodeReference(x,y-1);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x,y-1));

	element = node_manager_->GetNodeReference(x+1,y-1);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y-1));

	element = node_manager_->GetNodeReference(x-1,y);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x-1,y));

	element = node_manager_->GetNodeReference(x+1,y);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y));

	element = node_manager_->GetNodeReference(x-1,y+1);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x-1,y+1));

	element = node_manager_->GetNodeReference(x,y+1);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x,y+1));

	element = node_manager_->GetNodeReference(x+1,y+1);
	if(element != nullptr)
		dummy_neighbours.push_back(node_manager_->GetNodeReference(x+1,y+1));

	return dummy_neighbours;
}

std::vector<QuadTreeNode*> QuadTree::FindNeighbours(QuadTreeNode* node)
{
	std::vector<QuadTreeNode*> dummy_neighbours;
	std::vector<QuadTreeNode*> neighbours;

	// if the node is not at the highest resolution, find
	//	all its dummy leaf nodes
	int dummy_depth = 0;
	if(node->has_dummy_)
	{
		QuadTreeNode* node_index = node;

		// check how many levels of dummy nodes the node has
		while(node_index->node_type_!=NodeType::DUMMY_LEAF)
		{
			dummy_depth++;
			node_index = node_index->child_nodes_[0];
		}
	}

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

//	std::cout << "number of dummy neighbours: "<< dummy_neighbours.size() <<std::endl;

	// now find all dummy roots as neighbours in the quadtree
	neighbours.clear();
	std::vector<QuadTreeNode*>::iterator it;
	for(it = dummy_neighbours.begin(); it != dummy_neighbours.end(); ++it)
	{
		QuadTreeNode* qt_neighbour = (*it)->dummy_root_;

		bool existed = false;
		for(std::vector<QuadTreeNode*>::iterator it_qt = neighbours.begin(); it_qt != neighbours.end(); it_qt++)
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

void QTreeNodeManager::SetNodeReference(uint16_t index_x, uint16_t index_y, QuadTreeNode* node)
{
	tree_nodes_.at(index_y*side_node_num_ + index_x) = node;
}

QuadTreeNode* QTreeNodeManager::GetNodeReference(uint16_t index_x, uint16_t index_y)
{
	uint32_t index = index_y*side_node_num_ + index_x;
	if(index_x >= 0 && index_x < side_node_num_ &&
			index_y >= 0 && index_y < side_node_num_)
		return tree_nodes_.at(index_y*side_node_num_ + index_x);
	else
		return nullptr;
}

/*********************************************************/
/*              Implementation of Tree Node              */
/*********************************************************/

QuadTreeNode::QuadTreeNode(BoundingBox bound, OccupancyType occupancy):
		BDSBase<QuadTreeNode>(0),
		occupancy_(occupancy),dummy_root_(this),has_dummy_(false)
{
	node_type_ = NodeType::INNER;

	bounding_box_.x.min = bound.x.min;
	bounding_box_.x.max = bound.x.max;
	bounding_box_.y.min = bound.y.min;
	bounding_box_.y.max = bound.y.max;

	location_.x = bounding_box_.x.min + (bounding_box_.x.max - bounding_box_.x.min + 1)/2;
	location_.y = bounding_box_.y.min + (bounding_box_.y.max - bounding_box_.y.min + 1)/2;

	child_nodes_[0] = nullptr;
	child_nodes_[1] = nullptr;
	child_nodes_[2] = nullptr;
	child_nodes_[3] = nullptr;
}

QuadTreeNode::~QuadTreeNode()
{
}

bool QuadTreeNode::operator ==(const QuadTreeNode* other)
{
//	if(this->vertex_id_ != other->vertex_id_)
//	Should not use vertex_id_ since vertex_id_ is only assigned to leaf nodes
	if(this != other)
		return false;
	else
		return true;
}

double QuadTreeNode::GetHeuristic(const QuadTreeNode& other_struct) const {
	double x1,x2,y1,y2;

	x1 = other_struct.location_.x;
	y1 = other_struct.location_.y;

	x2 = other_struct.location_.x;
	y2 = other_struct.location_.y;

	// static_cast: can get wrong result to use "unsigned long" type for deduction
	long x_error = static_cast<long>(x1) - static_cast<long>(x2);
	long y_error = static_cast<long>(y1) - static_cast<long>(y2);

	double cost = std::abs(x_error) + std::abs(y_error);
//	std::cout<< "heuristic cost: " << cost << std::endl;

	return cost;
}
