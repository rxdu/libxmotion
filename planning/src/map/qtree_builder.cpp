#include <vector>
#include <cmath>

#include "qtree_builder.h"

using namespace srcl_ctrl;
using namespace cv;

QTreeBuilder::QTreeBuilder()
{

}

QTreeBuilder::~QTreeBuilder()
{
}

// TODO
// 1. Needs to handle src images of different sizes dynamically
// 2. Needs to check if src image is grayscale
bool QTreeBuilder::PreprocessImage(cv::InputArray _src)
{
	Mat image_bin;
	Mat src = _src.getMat();

	// Binarize grayscale image
	threshold(src, image_bin, 200, 255, THRESH_BINARY);

	// Pad image to be square
	bool pad_result = false;

	pad_result = PadGrayscaleImage(image_bin);

	return pad_result;
}

bool QTreeBuilder::PadGrayscaleImage(cv::InputArray _src)
{
	// create a image with size of power of 2
	Mat src = _src.getMat();

	unsigned long img_max_side;
	unsigned long padded_size = -1;

	if(src.cols > src.rows)
		img_max_side = src.cols;
	else
		img_max_side = src.rows;

	// find the minimal size of the padded image
	for(unsigned int i = 0; i <= 16; i++)
	{
		if((img_max_side > pow(2,i)) && (img_max_side <= pow(2, i+1)))
		{
			padded_size = pow(2, i+1);
			break;
		}
	}

	if(padded_size == -1)
		return false;

	std::cout << "padded size:" << padded_size << std::endl;

	padded_img_.create(padded_size,padded_size,0);
	Mat dst = padded_img_;

	int left, right, top, bottom;

	left = (dst.cols - src.cols)/2;
	right = dst.cols - src.cols - left;
	top = (dst.rows - src.rows)/2;
	bottom = dst.rows - src.rows - top;

	Scalar value = Scalar(0);
	copyMakeBorder(_src, dst, top, bottom, left, right, BORDER_CONSTANT,value);

	return true;

//	std::cout << "type - " << _src.type() << std::endl;
//	std::cout << "(cols, rows) = " << "(" << dst.cols << " , " << dst.rows << ")" << std::endl;
}

OccupancyType QTreeBuilder::CheckAreaOccupancy(BoundingBox area)
{
	Range rngx(area.x.min,area.x.max+1);
	Range rngy(area.y.min, area.y.max+1);

	// Attention:
	//	Points and Size go (x,y); (width,height) ,- Mat has (row,col).
	Mat checked_area = padded_img_(rngy,rngx);

	unsigned long free_points = 0;
	unsigned long occupied_points = 0;
	OccupancyType type;

	for(int i = 0; i < checked_area.cols; i++)
		for(int j = 0; j < checked_area.rows; j++)
		{
			if(checked_area.at<uchar>(Point(i,j)) > 0)
				free_points++;
			else
				occupied_points++;

			if(occupied_points !=0 && free_points != 0)
			{
				type = OccupancyType::MIXED;
				break;
			}
		}

	if(free_points !=0 && occupied_points == 0)
		type = OccupancyType::FREE;

	if(free_points ==0 && occupied_points != 0)
		type = OccupancyType::OCCUPIED;

	return type;

//	std::cout << "(cols, rows) = " << "(" << checked_area.cols << " , " << checked_area.rows << ")" << std::endl;
}

/*
 * @param _src: grayscale image, max size: 2^16 * 2^16 = 65535 * 65535 pixels
 * @param max_depth: maximum depth of the tree to be built
 */
QuadTree* QTreeBuilder::BuildQuadTree(cv::InputArray _src, unsigned int max_depth)
{
	// Pad image to get a image with size of power of 2
	//	src_img_ can be used only after this step
	if(!PreprocessImage(_src))
		return nullptr;

	// Create quadtree
	QuadTree *tree_ = new QuadTree(padded_img_.cols, max_depth);

	if(max_depth > tree_->MAX_DEPTH)
	{
		max_depth = tree_->MAX_DEPTH;
		std::cout << "Maximum depth allowed is 32. Only 32 levels will be built." << std::endl;
	}

	std::vector<QuadTreeNode*> parent_nodes;

	for(int grown_depth = 0; grown_depth <= max_depth; grown_depth++)
	{
#ifdef DEBUG
		std::cout << "growth level:" << grown_depth << std::endl;
#endif
		// root level
		if(grown_depth == 0)
		{
			BoundingBox bbox;
			bbox.x.min = 0;
			bbox.x.max = padded_img_.cols - 1;
			bbox.y.min = 0;
			bbox.y.max = padded_img_.rows - 1;

			OccupancyType map_occupancy;
			map_occupancy = CheckAreaOccupancy(bbox);

			tree_->root_node_ = new QuadTreeNode(bbox, map_occupancy);

			// if map is empty, terminate the process
			if(map_occupancy != OccupancyType::MIXED)
			{
				tree_->root_node_->node_type_ = NodeType::LEAF;
				return tree_;
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes.push_back(tree_->root_node_);
		}
		// lower levels
		else
		{
			std::vector<QuadTreeNode*> inner_nodes;

			while(!parent_nodes.empty())
			{
				// divide the parent area
				QuadTreeNode* parent = parent_nodes.at(0);

				/* opencv image coordinate:
				 * 	0 - > x
				 * 	|
				 * 	v
				 * 	y
				 */
				BoundingBox bbox[4];
				OccupancyType occupancy[4];

				// top left area
				bbox[0].x.min = parent->bounding_box_.x.min;
				bbox[0].x.max = parent->bounding_box_.x.min + (parent->bounding_box_.x.max - parent->bounding_box_.x.min + 1)/2 - 1;
				bbox[0].y.min = parent->bounding_box_.y.min;
				bbox[0].y.max = parent->bounding_box_.y.min + (parent->bounding_box_.y.max - parent->bounding_box_.y.min + 1)/2 - 1;

				// top right area
				bbox[1].x.min = bbox[0].x.max + 1;
				bbox[1].x.max = parent->bounding_box_.x.max;
				bbox[1].y.min = bbox[0].y.min;
				bbox[1].y.max = bbox[0].y.max;

				// bottom left area
				bbox[2].x.min = bbox[0].x.min;
				bbox[2].x.max = bbox[0].x.max;
				bbox[2].y.min = bbox[0].y.max + 1;
				bbox[2].y.max = parent->bounding_box_.y.max;

				// bottom right area
				bbox[3].x.min = bbox[1].x.min;
				bbox[3].x.max = bbox[1].x.max;
				bbox[3].y.min = bbox[2].y.min;
				bbox[3].y.max = bbox[2].y.max;

				for(int i = 0; i < 4; i++)
				{
#ifdef DEBUG
					std::cout << "bounding box " << i <<": " << "x " << bbox[i].x.min << "-" << bbox[i].x.max
							<< " y " << bbox[i].y.min << "-" << bbox[i].y.max << std::endl;
#endif

					occupancy[i] = CheckAreaOccupancy(bbox[i]);
					parent->child_nodes_[i] = new QuadTreeNode(bbox[i], occupancy[i]);

					if(grown_depth < max_depth)
					{
						// first push back this node for further dividing
						inner_nodes.push_back(parent->child_nodes_[i]);

						// check and assign node type
						if(occupancy[i] != OccupancyType::MIXED)
						{
							if(parent->node_type_ != NodeType::INNER)
							{
								parent->child_nodes_[i]->node_type_ = NodeType::DUMMY_INNER;

								if(parent->node_type_ == NodeType::LEAF)
								{
									parent->child_nodes_[i]->dummy_root_ = parent;
									parent->has_dummy_ = true;
								}
								else
									parent->child_nodes_[i]->dummy_root_ = parent->dummy_root_;
							}
							else
								parent->child_nodes_[i]->node_type_ = NodeType::LEAF;
						}
					}
					else
					{
						// assign node type as leaf
						if(parent->node_type_ != NodeType::INNER)
						{
							parent->child_nodes_[i]->node_type_ = NodeType::DUMMY_LEAF;

							if(parent->node_type_ == NodeType::LEAF)
							{
								parent->child_nodes_[i]->dummy_root_ = parent;
								parent->has_dummy_ = true;
							}
							else
								parent->child_nodes_[i]->dummy_root_ = parent->dummy_root_;
						}
						else
							parent->child_nodes_[i]->node_type_ = NodeType::LEAF;

						// generate index for the node
						uint16_t x,y;
						x = ((bbox[i].x.min + bbox[i].x.max + 1)/2)/tree_->cell_res_;
						y = ((bbox[i].y.min + bbox[i].y.max + 1)/2)/tree_->cell_res_;
						tree_->node_manager_->SetNodeReference(x,y,parent->child_nodes_[i]);
					}
				}

				// delete the processed node
				parent_nodes.erase(parent_nodes.begin());
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes = inner_nodes;
		}

		tree_->tree_depth_++;
	}

	// Store all leaf nodes into a vector
	tree_->leaf_nodes_ = GetAllLeafNodes(tree_);

	return tree_;
}

std::vector<QuadTreeNode*> QTreeBuilder::GetAllLeafNodes(QuadTree *tree)
{
	std::vector<QuadTreeNode*> leaves;

	if(tree != nullptr)
	{
		std::vector<QuadTreeNode*> parent_nodes;

		for(int i = 0; i < tree->tree_depth_; i++)
		{
			if(i == 0)
			{
				if(tree->root_node_->node_type_ != NodeType::LEAF)
				{
					for(int i = 0; i < 4; i++)
					{
						parent_nodes.clear();
						parent_nodes.push_back(tree->root_node_);
					}
				}
				else
				{
					leaves.push_back(tree->root_node_);
					break;
				}
			}
			else
			{
				std::vector<QuadTreeNode*> inner_nodes;

				while(!parent_nodes.empty())
				{
					QuadTreeNode* parent = parent_nodes.at(0);

					for(int i = 0; i < 4; i++)
					{
						if(parent->child_nodes_[i]->node_type_ == NodeType::INNER)
							inner_nodes.push_back(parent->child_nodes_[i]);
						else if(parent->child_nodes_[i]->node_type_ == NodeType::LEAF)
							leaves.push_back(parent->child_nodes_[i]);
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

	// Assign id to tree leaf nodes
	std::vector<QuadTreeNode*>::iterator it;
	uint64_t id = 0;
	for(it = leaves.begin(); it != leaves.end(); it++)
	{
		(*it)->node_id_ = id;
		id++;
	}

	return leaves;
}
