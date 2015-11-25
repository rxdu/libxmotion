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
