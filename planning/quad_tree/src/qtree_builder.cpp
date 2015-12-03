#include <vector>

#include "qtree_builder.h"

using namespace srcl_ctrl;
using namespace cv;

QTreeBuilder::QTreeBuilder():tree_(nullptr)
{

}

QTreeBuilder::~QTreeBuilder()
{
	if(tree_ != nullptr)
		delete(tree_);
}

// TODO
// 1. Needs to handle src images of different sizes dynamically
// 2. Needs to check if src image is grayscale
void QTreeBuilder::PadGrayscaleImage(cv::InputArray _src, cv::OutputArray _dst)
{
	// create a image with size of power of 2
	Mat src = _src.getMat();
	_dst.create(1024,1024,0);
	Mat dst = _dst.getMat();

	int left, right, top, bottom;

	left = (dst.cols - src.cols)/2;
	right = dst.cols - src.cols - left;
	top = (dst.rows - src.rows)/2;
	bottom = dst.rows - src.rows - top;

	Scalar value = Scalar(0);
	copyMakeBorder(_src, dst, top, bottom, left, right, BORDER_CONSTANT,value);

//	std::cout << "type - " << _src.type() << std::endl;
//	std::cout << "(cols, rows) = " << "(" << dst.cols << " , " << dst.rows << ")" << std::endl;
}

void QTreeBuilder::PadGrayscaleImage(cv::InputArray _src)
{
	// create a image with size of power of 2
	Mat src = _src.getMat();
	src_img_.create(1024,1024,0);
	Mat dst = src_img_;

	int left, right, top, bottom;

	left = (dst.cols - src.cols)/2;
	right = dst.cols - src.cols - left;
	top = (dst.rows - src.rows)/2;
	bottom = dst.rows - src.rows - top;

	Scalar value = Scalar(0);
	copyMakeBorder(_src, dst, top, bottom, left, right, BORDER_CONSTANT,value);

//	std::cout << "type - " << _src.type() << std::endl;
//	std::cout << "(cols, rows) = " << "(" << dst.cols << " , " << dst.rows << ")" << std::endl;
}

OccupancyType QTreeBuilder::CheckAreaOccupancy(BoundingBox area)
{
	Mat checked_area = src_img_(Range(area.x.min,area.x.max+1),Range(area.y.min, area.y.max+1));

//	std::cout << "(cols, rows) = " << "(" << checked_area.cols << " , " << checked_area.rows << ")" << std::endl;

	return OccupancyType::MIXED;
}

void QTreeBuilder::BuildQuadTree(cv::InputArray _src, unsigned int max_depth)
{
	// Pad image to get a image with size of power of 2
	//	src_img_ can be used only after this step
	PadGrayscaleImage(_src);

	// Create quadtree
	tree_ = new QuadTree();

	std::vector<TreeNode*> parent_nodes;

	for(int grown_depth = 0; grown_depth < max_depth; grown_depth++)
	{
		std::cout << "growth level:" << grown_depth << std::endl;

		// root level
		if(grown_depth == 0)
		{
			BoundingBox bbox;
			bbox.x.min = 0;
			bbox.x.max = src_img_.cols - 1;
			bbox.y.min = 0;
			bbox.y.max = src_img_.rows - 1;

			OccupancyType map_occupancy;
			map_occupancy = CheckAreaOccupancy(bbox);

			tree_->root_node_ = new TreeNode(bbox, map_occupancy);

			// if map is empty, terminate the process
			if(map_occupancy != OccupancyType::MIXED)
			{
				tree_->root_node_->type_ = NodeType::LEAF;
				return;
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes.push_back(tree_->root_node_);
		}
		// lower levels
		else
		{
			std::vector<TreeNode*> inner_nodes;

			while(!parent_nodes.empty())
			{
				// divide the parent area
				TreeNode* parent = parent_nodes.at(0);

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
				bbox[0].x.max = (parent->bounding_box_.x.max + 1)/2 - 1;
				bbox[0].y.min = parent->bounding_box_.y.min;
				bbox[0].y.max = (parent->bounding_box_.y.max + 1)/2 - 1;

				// top right area
				bbox[1].x.min = (parent->bounding_box_.x.max + 1)/2;
				bbox[1].x.max = parent->bounding_box_.x.max;
				bbox[1].y.min = parent->bounding_box_.y.min;
				bbox[1].y.max = (parent->bounding_box_.y.max + 1)/2 - 1;

				// bottom left area
				bbox[2].x.min = parent->bounding_box_.x.min;
				bbox[2].x.max = (parent->bounding_box_.x.max + 1)/2 - 1;
				bbox[2].y.min =(parent->bounding_box_.y.max + 1)/2;
				bbox[2].y.max = parent->bounding_box_.y.max;

				// bottom right area
				bbox[3].x.min = (parent->bounding_box_.x.max + 1)/2;
				bbox[3].x.max = parent->bounding_box_.x.max;
				bbox[3].y.min =(parent->bounding_box_.y.max + 1)/2;
				bbox[3].y.max = parent->bounding_box_.y.max;

				for(int i = 0; i < 4; i++)
				{
					occupancy[i] = CheckAreaOccupancy(bbox[i]);
					parent->child_nodes_[i] = new TreeNode(bbox[i], occupancy[i]);

					if(grown_depth < max_depth - 1)
					{
						if(occupancy[i] != OccupancyType::MIXED)
							parent->child_nodes_[i]->type_ = NodeType::LEAF;
						else
							inner_nodes.push_back(parent->child_nodes_[i]);
					}
					else
						parent->child_nodes_[i]->type_ = NodeType::LEAF;

					std::cout << "bounding box " << i <<": " << "x " << bbox[i].x.min << "-" << bbox[i].x.max
											<< " y " << bbox[i].y.min << "-" << bbox[i].y.max << std::endl;
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
