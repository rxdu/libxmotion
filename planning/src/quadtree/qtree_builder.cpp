#include <vector>
#include <cmath>

#include "qtree_builder.h"

using namespace srcl_ctrl;
using namespace cv;

QTreeBuilder::QTreeBuilder():
		tree_(nullptr),exttree_(nullptr)
{

}

QTreeBuilder::~QTreeBuilder()
{
	if(tree_ != nullptr)
		delete(tree_);
	if(exttree_ != nullptr)
		delete(exttree_);
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

	src_img_.create(padded_size,padded_size,0);
	Mat dst = src_img_;

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
	Mat checked_area = src_img_(rngy,rngx);

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
void QTreeBuilder::BuildQuadTree(cv::InputArray _src, unsigned int max_depth)
{
	// Pad image to get a image with size of power of 2
	//	src_img_ can be used only after this step
	if(!PreprocessImage(_src))
		return;

	// Create quadtree
	tree_ = new QuadTree();

	if(max_depth > tree_->MAX_DEPTH)
	{
		max_depth = tree_->MAX_DEPTH;
		std::cout << "Maximum depth allowed is 32. Only 32 levels will be built." << std::endl;
	}

	std::vector<TreeNode*> parent_nodes;

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
			bbox.x.max = src_img_.cols - 1;
			bbox.y.min = 0;
			bbox.y.max = src_img_.rows - 1;

			OccupancyType map_occupancy;
			map_occupancy = CheckAreaOccupancy(bbox);

			tree_->root_node_ = new TreeNode(bbox, map_occupancy);

			// if map is empty, terminate the process
			if(map_occupancy != OccupancyType::MIXED)
			{
				tree_->root_node_->node_type_ = NodeType::LEAF;
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
					parent->child_nodes_[i] = new TreeNode(bbox[i], occupancy[i]);

					if(grown_depth < max_depth)
					{
						if(occupancy[i] != OccupancyType::MIXED)
							parent->child_nodes_[i]->node_type_ = NodeType::LEAF;
						else
							inner_nodes.push_back(parent->child_nodes_[i]);
					}
					else
						parent->child_nodes_[i]->node_type_ = NodeType::LEAF;
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
}


/*
 * @param _dst: an Mat object that points to the result image
 * @param vis_type: FREE_SPACE, OCCU_SPACE or ALL_SPACE
 */
void QTreeBuilder::VisualizeQuadTree(cv::OutputArray _dst, TreeVisType vis_type)
{
	Mat src_img_color;
	cvtColor(src_img_, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	if(tree_ != nullptr)
	{
		std::vector<TreeNode*> parent_nodes;

		for(int i = 0; i < tree_->tree_depth_; i++)
		{
			if(i == 0)
			{
				Point top_left(tree_->root_node_->bounding_box_.x.min, tree_->root_node_->bounding_box_.y.min);
				Point top_right(tree_->root_node_->bounding_box_.x.max,tree_->root_node_->bounding_box_.y.min);
				Point bot_left(tree_->root_node_->bounding_box_.x.min,tree_->root_node_->bounding_box_.y.max);
				Point bot_right(tree_->root_node_->bounding_box_.x.max,tree_->root_node_->bounding_box_.y.max);

				line(src_img_color, top_left, top_right, Scalar(0,255,0));
				line(src_img_color, top_right, bot_right, Scalar(0,255,0));
				line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
				line(src_img_color, bot_left, top_left, Scalar(0,255,0));

				if(tree_->root_node_->node_type_ != NodeType::LEAF)
				{
					for(int i = 0; i < 4; i++)
					{
						parent_nodes.clear();
						parent_nodes.push_back(tree_->root_node_);
					}
				}
				else
					break;
			}
			else
			{
				std::vector<TreeNode*> inner_nodes;

				while(!parent_nodes.empty())
				{
					TreeNode* parent = parent_nodes.at(0);

					for(int i = 0; i < 4; i++)
					{
						if(vis_type == TreeVisType::ALL_SPACE)
						{
							Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
							Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
							Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
							Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

							line(src_img_color, top_left, top_right, Scalar(0,255,0));
							line(src_img_color, top_right, bot_right, Scalar(0,255,0));
							line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
							line(src_img_color, bot_left, top_left, Scalar(0,255,0));

//							if(parent->child_nodes_[i]->type_ == NodeType::LEAF)
//							{
//								int thickness = -1;
//								int lineType = 8;
//								unsigned int x,y;
//								x = parent->child_nodes_[i]->bounding_box_.x.min +
//										(parent->child_nodes_[i]->bounding_box_.x.max - parent->child_nodes_[i]->bounding_box_.x.min + 1)/2;
//								y = parent->child_nodes_[i]->bounding_box_.y.min +
//										(parent->child_nodes_[i]->bounding_box_.y.max - parent->child_nodes_[i]->bounding_box_.y.min + 1)/2;
//								Point center(x,y);
//								circle( src_img_color,
//										center,
//										5,
//										Scalar( 0, 0, 255 ),
//										thickness,
//										lineType);
//							}
						}
						else
						{
							OccupancyType disp_type;

							if(vis_type == TreeVisType::FREE_SPACE)
								disp_type = OccupancyType::FREE;
							else
								disp_type = OccupancyType::OCCUPIED;

							if(parent->child_nodes_[i]->occupancy_ == disp_type){
								Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
								Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
								Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
								Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

								line(src_img_color, top_left, top_right, Scalar(0,255,0));
								line(src_img_color, top_right, bot_right, Scalar(0,255,0));
								line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
								line(src_img_color, bot_left, top_left, Scalar(0,255,0));

//								if(parent->child_nodes_[i]->type_ == NodeType::LEAF)
//								{
//									int thickness = -1;
//									int lineType = 8;
//									unsigned int x,y;
//									x = parent->child_nodes_[i]->bounding_box_.x.min +
//											(parent->child_nodes_[i]->bounding_box_.x.max - parent->child_nodes_[i]->bounding_box_.x.min + 1)/2;
//									y = parent->child_nodes_[i]->bounding_box_.y.min +
//											(parent->child_nodes_[i]->bounding_box_.y.max - parent->child_nodes_[i]->bounding_box_.y.min + 1)/2;
//									Point center(x,y);
//									circle( src_img_color,
//											center,
//											5,
//											Scalar( 0, 0, 255 ),
//											thickness,
//											lineType);
//								}
							}
						}

						if(parent->child_nodes_[i]->node_type_ == NodeType::LEAF && parent->child_nodes_[i]->occupancy_ == OccupancyType::FREE)
						{
							int thickness = -1;
							int lineType = 8;
							unsigned int x,y;
							x = parent->child_nodes_[i]->bounding_box_.x.min +
									(parent->child_nodes_[i]->bounding_box_.x.max - parent->child_nodes_[i]->bounding_box_.x.min + 1)/2;
							y = parent->child_nodes_[i]->bounding_box_.y.min +
									(parent->child_nodes_[i]->bounding_box_.y.max - parent->child_nodes_[i]->bounding_box_.y.min + 1)/2;
							Point center(x,y);
							circle( src_img_color,
									center,
									5,
									Scalar( 0, 0, 255 ),
									thickness,
									lineType);
						}

						if(parent->child_nodes_[i]->node_type_ != NodeType::LEAF)
							inner_nodes.push_back(parent->child_nodes_[i]);
					}

					// delete the processed node
					parent_nodes.erase(parent_nodes.begin());
				}

				// prepare for next iteration
				parent_nodes.clear();
				parent_nodes = inner_nodes;
			}
		}
//		std::cout << "image size: "<<dst.cols << ","<< dst.rows<<std::endl;
	}

	vis_img_ = src_img_color;
	src_img_color.copyTo(dst);
}

/*
 * @param _src: grayscale image, max size: 2^16 * 2^16 = 65535 * 65535 pixels
 * @param max_depth: maximum depth of the tree to be built
 */
void QTreeBuilder::BuildExtQuadTree(cv::InputArray _src, unsigned int max_depth)
{
	// Pad image to get a image with size of power of 2
	//	src_img_ can be used only after this step
	if(!PreprocessImage(_src))
		return;

	// Create quadtree
	exttree_ = new QuadTree(src_img_.cols, max_depth);

	if(max_depth > exttree_->MAX_DEPTH)
	{
		max_depth = exttree_->MAX_DEPTH;
		std::cout << "Maximum depth allowed is 32. Only 32 levels will be built." << std::endl;
	}

	std::vector<TreeNode*> parent_nodes;

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
			bbox.x.max = src_img_.cols - 1;
			bbox.y.min = 0;
			bbox.y.max = src_img_.rows - 1;

			OccupancyType map_occupancy;
			map_occupancy = CheckAreaOccupancy(bbox);

			exttree_->root_node_ = new TreeNode(bbox, map_occupancy);

			// if map is empty, terminate the process
			if(map_occupancy != OccupancyType::MIXED)
			{
				exttree_->root_node_->node_type_ = NodeType::LEAF;
				return;
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes.push_back(exttree_->root_node_);
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
					parent->child_nodes_[i] = new TreeNode(bbox[i], occupancy[i]);

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
//						parent->child_nodes_[i]->node_type_ = NodeType::LEAF;

						// generate index for the node
						uint16_t x,y;
						x = ((bbox[i].x.min + bbox[i].x.max + 1)/2)/exttree_->cell_res_;
						y = ((bbox[i].y.min + bbox[i].y.max + 1)/2)/exttree_->cell_res_;
//						std::cout << "bounding box " << i <<": " << "x " << bbox[i].x.min << "-" << bbox[i].x.max
//								<< " y " << bbox[i].y.min << "-" << bbox[i].y.max << std::endl;
//						std::cout << "cell res: "<<exttree_->cell_res_<<std::endl;
//						std::cout << "coordinate: x - "<< x << " y - "<< y << std::endl;
						exttree_->node_manager_->SetNodeReference(x,y,parent->child_nodes_[i]);
					}
				}

				// delete the processed node
				parent_nodes.erase(parent_nodes.begin());
			}

			// prepare for next iteration
			parent_nodes.clear();
			parent_nodes = inner_nodes;
		}

		exttree_->tree_depth_++;
	}
}

void QTreeBuilder::VisualizeExtQuadTree(cv::OutputArray _dst, TreeVisType vis_type)
{
	Mat src_img_color;
	cvtColor(src_img_, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	if(exttree_ != nullptr)
	{
		std::vector<TreeNode*> parent_nodes;

		for(int i = 0; i < exttree_->tree_depth_; i++)
		{
			if(i == 0)
			{
				Point top_left(exttree_->root_node_->bounding_box_.x.min, exttree_->root_node_->bounding_box_.y.min);
				Point top_right(exttree_->root_node_->bounding_box_.x.max,exttree_->root_node_->bounding_box_.y.min);
				Point bot_left(exttree_->root_node_->bounding_box_.x.min,exttree_->root_node_->bounding_box_.y.max);
				Point bot_right(exttree_->root_node_->bounding_box_.x.max,exttree_->root_node_->bounding_box_.y.max);

				line(src_img_color, top_left, top_right, Scalar(0,255,0));
				line(src_img_color, top_right, bot_right, Scalar(0,255,0));
				line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
				line(src_img_color, bot_left, top_left, Scalar(0,255,0));

				if(exttree_->root_node_->node_type_ != NodeType::LEAF)
				{
					for(int i = 0; i < 4; i++)
					{
						parent_nodes.clear();
						parent_nodes.push_back(exttree_->root_node_);
					}
				}
				else
					break;
			}
			else
			{
				std::vector<TreeNode*> inner_nodes;

				while(!parent_nodes.empty())
				{
					TreeNode* parent = parent_nodes.at(0);

					for(int i = 0; i < 4; i++)
					{
						if(vis_type == TreeVisType::ALL_SPACE)
						{
							Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
							Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
							Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
							Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

							line(src_img_color, top_left, top_right, Scalar(0,255,0));
							line(src_img_color, top_right, bot_right, Scalar(0,255,0));
							line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
							line(src_img_color, bot_left, top_left, Scalar(0,255,0));
						}
						else
						{
							OccupancyType disp_type;

							if(vis_type == TreeVisType::FREE_SPACE)
								disp_type = OccupancyType::FREE;
							else
								disp_type = OccupancyType::OCCUPIED;

							if(parent->child_nodes_[i]->occupancy_ == disp_type){
								Point top_left(parent->child_nodes_[i]->bounding_box_.x.min, parent->child_nodes_[i]->bounding_box_.y.min);
								Point top_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.min);
								Point bot_left(parent->child_nodes_[i]->bounding_box_.x.min,parent->child_nodes_[i]->bounding_box_.y.max);
								Point bot_right(parent->child_nodes_[i]->bounding_box_.x.max,parent->child_nodes_[i]->bounding_box_.y.max);

								line(src_img_color, top_left, top_right, Scalar(0,255,0));
								line(src_img_color, top_right, bot_right, Scalar(0,255,0));
								line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
								line(src_img_color, bot_left, top_left, Scalar(0,255,0));
							}
						}

#ifdef DEBUG
						if(parent->child_nodes_[i]->node_type_ == NodeType::LEAF && parent->child_nodes_[i]->occupancy_ == OccupancyType::FREE)
						{
							int thickness = -1;
							int lineType = 8;
							unsigned int x,y;
							x = parent->child_nodes_[i]->bounding_box_.x.min +
									(parent->child_nodes_[i]->bounding_box_.x.max - parent->child_nodes_[i]->bounding_box_.x.min + 1)/2;
							y = parent->child_nodes_[i]->bounding_box_.y.min +
									(parent->child_nodes_[i]->bounding_box_.y.max - parent->child_nodes_[i]->bounding_box_.y.min + 1)/2;
							Point center(x,y);
							circle( src_img_color,
									center,
									5,
									Scalar( 0, 0, 255 ),
									thickness,
									lineType);
						}
#endif
						if(parent->child_nodes_[i]->node_type_ == NodeType::INNER)
							inner_nodes.push_back(parent->child_nodes_[i]);
					}

					// delete the processed node
					parent_nodes.erase(parent_nodes.begin());
				}

				// prepare for next iteration
				parent_nodes.clear();
				parent_nodes = inner_nodes;
			}
		}

#ifdef DEBUG
		std::cout << "side node number from manager: " << exttree_->node_manager_->side_node_num_<<std::endl;
		for(int i = 0; i < exttree_->node_manager_->side_node_num_; i++)
			for(int j = 0; j < exttree_->node_manager_->side_node_num_; j++)
			{
//				std::cout << "i,j = " << i << "," << j << std::endl;
				TreeNode* node = exttree_->node_manager_->GetNodeReference(i,j);

				if(node->node_type_ == NodeType::LEAF)
				{
					int thickness = -1;
					int lineType = 8;
					unsigned int x,y;
					x = node->bounding_box_.x.min +
							(node->bounding_box_.x.max - node->bounding_box_.x.min + 1)/2;
					y = node->bounding_box_.y.min +
							(node->bounding_box_.y.max - node->bounding_box_.y.min + 1)/2;
					Point center(x,y);
					circle( src_img_color,
							center,
							5,
							Scalar( 0, 0, 255 ),
							thickness,
							lineType);
				}
			}
#endif

		// Code to visualize neighbour finding
//		TreeNode* node = exttree_->GetNodeAtPosition(0,1)->dummy_root_;
//		TreeNode* node = exttree_->GetNodeAtPosition(2047,1)->dummy_root_;
//		TreeNode* node = exttree_->GetNodeAtPosition(0,2047)->dummy_root_;
//		TreeNode* node = exttree_->GetNodeAtPosition(2047,2047)->dummy_root_;
//		TreeNode* node = exttree_->GetNodeAtPosition(1312,1024)->dummy_root_;
//		TreeNode* node = exttree_->GetNodeAtPosition(1023,1024)->dummy_root_;
		TreeNode* node = exttree_->GetNodeAtPosition(1023,1100)->dummy_root_;

		if(node!=nullptr)
		{
			int thickness = -1;
			int lineType = 8;
			unsigned int x,y;
			x = node->bounding_box_.x.min +
					(node->bounding_box_.x.max - node->bounding_box_.x.min + 1)/2;
			y = node->bounding_box_.y.min +
					(node->bounding_box_.y.max - node->bounding_box_.y.min + 1)/2;
			Point center(x,y);
			circle( src_img_color,
					center,
					10,
					Scalar( 0, 0, 255 ),
					thickness,
					lineType);
			std::vector<TreeNode*> node_list;

			std::cout << "Trying to find neighbours"<<std::endl;
			node_list = exttree_->FindNeighbours(node);

			std::cout << "checked node num: "<<node_list.size()<<std::endl;

			while(!node_list.empty())
			{
				TreeNode* dispnode = node_list.at(0);

				x = dispnode->bounding_box_.x.min +
						(dispnode->bounding_box_.x.max - dispnode->bounding_box_.x.min + 1)/2;
				y = dispnode->bounding_box_.y.min +
						(dispnode->bounding_box_.y.max - dispnode->bounding_box_.y.min + 1)/2;
				Point center(x,y);
				circle( src_img_color,
						center,
						5,
						Scalar( 0, 128, 255 ),
						thickness,
						lineType);

				node_list.erase(node_list.begin());
			}
		}
		else
			std::cout << "Node to be checked is empty"<<std::endl;

		//		std::cout << "image size: "<<dst.cols << ","<< dst.rows<<std::endl;
	}

	vis_img_ = src_img_color;
	src_img_color.copyTo(dst);
}
