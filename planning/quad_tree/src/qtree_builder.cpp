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

	std::cout << "type - " << _src.type() << std::endl;
	std::cout << "(cols, rows) = " << "(" << dst.cols << " , " << dst.rows << ")" << std::endl;
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

	std::cout << "type - " << _src.type() << std::endl;
	std::cout << "(cols, rows) = " << "(" << dst.cols << " , " << dst.rows << ")" << std::endl;
}

bool QTreeBuilder::CheckAreaOccupancy(unsigned int x_start, unsigned int x_end, unsigned int y_start, unsigned int y_end)
{
	Mat checked_area = src_img_(Range(x_start,x_end),Range(y_start, y_end));
}

void QTreeBuilder::BuildQuadTree(cv::InputArray _src, unsigned int max_depth)
{
	// Pad image to get a image with size of power of 2
	//	src_img_ can be used only after this step
	PadGrayscaleImage(_src);

	// Create quadtree
	BoundingBox root_bb;
	root_bb.x.min = 0;
	root_bb.x.max = src_img_.cols;
	root_bb.y.min = 0;
	root_bb.y.max = src_img_.rows;
	// TODO
	// Check if the root area is occupied
	tree_ = new QuadTree(root_bb, true);

//	for(int grown_depth = 1; grown_depth < max_depth; grown_depth++)
//	{
//		// divide the parent nodes at level grown_depth-1
//		for(int i = 0; i < 4; i++)
//		{
//
//		}
//	}

	for(int i = 0; i < 4; i++)
	{

//		tree_->root_node_->child_nodes_[i] =
	}
}
